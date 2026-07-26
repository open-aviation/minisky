import type { Entity, PluginContext } from "@open-aviation/tangram-core/api";
import {
  TrajectoryApi,
  type BusResultEnvelope,
  type EntityKey,
  type TrajectoryGetRequest,
  type TrajectoryGetResult
} from "@open-aviation/tangram-core/api";
import AircraftLayer from "./AircraftLayer.vue";
import AircraftTrailLayer from "./AircraftTrailLayer.vue";
import AircraftCountWidget from "./AircraftCountWidget.vue";
import SimControlWidget from "./SimControlWidget.vue";
import { miniskyStore, pushLog, type MiniskyAircraft, type SimInfo } from "./store";

export const ENTITY_TYPE = "minisky_aircraft";
const SOURCE = "tangram_minisky";

// If no snapshot or heartbeat arrives for this long, the simulator is gone.
// The producer heartbeats every second even while paused.
const STALE_AFTER_MS = 5000;

interface MiniskyFrontendConfig {
  channel: string;
  topbar_order: number;
  sidebar_order: number;
}

interface StreamPayload {
  aircraft: MiniskyAircraft[];
  count: number;
  siminfo: SimInfo;
}

export function install(ctx: PluginContext, config?: MiniskyFrontendConfig) {
  const api = ctx.api;
  const channel = config?.channel || "minisky";
  miniskyStore.channel = channel;

  api.state.registerEntityType(ENTITY_TYPE, { pluginId: ctx.id });

  api.ui.registerWidget("minisky-count-widget", "TopBar", AircraftCountWidget, {
    pluginId: ctx.id,
    priority: config?.topbar_order
  });
  api.ui.registerWidget("minisky-aircraft-layer", "MapOverlay", AircraftLayer, {
    pluginId: ctx.id
  });
  api.ui.registerWidget("minisky-trail-layer", "MapOverlay", AircraftTrailLayer, {
    pluginId: ctx.id
  });
  api.ui.registerWidget("minisky-control-widget", "SideBar", SimControlWidget, {
    pluginId: ctx.id,
    priority: config?.sidebar_order,
    title: "MiniSky Simulator"
  });

  const staleTimer = setInterval(() => {
    if (miniskyStore.connected && Date.now() - miniskyStore.lastUpdate > STALE_AFTER_MS) {
      miniskyStore.connected = false;
    }
  }, 1000);
  ctx.onDispose({ dispose: () => clearInterval(staleTimer) });

  // Trajectories are tracked for selected aircraft only and shared through
  // the core trajectory store (api.bus / api.trajectory), so other plugins
  // can consume the simulator feed without depending on this package.
  let selectedIds: ReadonlySet<string> = new Set();

  const trajectoryKey = (id: string): EntityKey => ({ id, type: ENTITY_TYPE });

  ctx.onDispose(
    api.selection.onChanged(selection => {
      const next = selection.get(ENTITY_TYPE) ?? new Set<string>();
      for (const id of next) {
        if (selectedIds.has(id)) continue;
        const entity = api.state.getEntity<MiniskyAircraft>(trajectoryKey(id));
        const points =
          entity && entity.state.latitude != null && entity.state.longitude != null
            ? [entity.state]
            : [];
        api.bus.publish(TrajectoryApi.TOPIC_INIT, {
          key: trajectoryKey(id),
          points,
          source: SOURCE
        });
      }
      selectedIds = next;
    })
  );

  // Serve trajectory requests from other plugins (bus authority).
  ctx.onDispose(
    api.bus.subscribe<TrajectoryGetRequest>(TrajectoryApi.TOPIC_GET, req => {
      if (req.key.type !== ENTITY_TYPE) return;
      api.bus.publish<BusResultEnvelope<TrajectoryGetResult>>(
        `${TrajectoryApi.TOPIC_GET}:result`,
        {
          request_id: req.request_id,
          data: {
            key: req.key,
            points: api.trajectory.get(req.key).value,
            source: SOURCE
          }
        }
      );
    })
  );

  let lastSimt = Number.NEGATIVE_INFINITY;

  void (async () => {
    try {
      await api.realtime.ensureConnected();

      ctx.onDispose(
        await api.realtime.subscribe<StreamPayload>(`${channel}:new-data`, payload => {
          // Simulation reset (or a new scenario): drop stale trails.
          if (payload.siminfo.simt < lastSimt) {
            for (const id of selectedIds) {
              api.bus.publish(TrajectoryApi.TOPIC_INIT, {
                key: trajectoryKey(id),
                points: [],
                source: SOURCE
              });
            }
          }
          lastSimt = payload.siminfo.simt;

          const entities: Entity[] = payload.aircraft
            .filter(ac => ac.latitude != null && ac.longitude != null)
            .map(ac => ({ id: ac.id, type: ENTITY_TYPE, state: ac }));
          api.state.replaceAllEntitiesByType(ENTITY_TYPE, entities);
          api.state.setTotalCount(ENTITY_TYPE, payload.count);

          miniskyStore.siminfo = payload.siminfo;
          miniskyStore.connected = true;
          miniskyStore.lastUpdate = Date.now();

          for (const id of selectedIds) {
            const ac = payload.aircraft.find(a => a.id === id);
            if (!ac || ac.latitude == null || ac.longitude == null) continue;
            const trajectory = api.trajectory.get(trajectoryKey(id)).value;
            const last = trajectory[trajectory.length - 1] as
              | MiniskyAircraft
              | undefined;
            // Heartbeats repeat the last snapshot; skip duplicate points.
            if (ac.timestamp != null && last?.timestamp === ac.timestamp) continue;
            api.bus.publish(TrajectoryApi.TOPIC_APPEND, {
              key: trajectoryKey(id),
              points: [ac],
              source: SOURCE
            });
          }
        })
      );

      ctx.onDispose(
        await api.realtime.subscribe<{ lines: string[] }>(
          `${channel}:console`,
          payload => {
            for (const line of payload.lines ?? []) {
              pushLog("msg", line);
            }
          }
        )
      );
    } catch (e) {
      console.error("tangram_minisky: realtime subscription failed", e);
    }
  })();
}
