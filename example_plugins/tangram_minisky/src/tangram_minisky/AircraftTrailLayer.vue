<template>
  <!-- renders only into the deck.gl overlay -->
  <span style="display: none"></span>
</template>

<script setup lang="ts">
import { inject, onUnmounted, ref, watchEffect, type Ref } from "vue";
import { PathLayer } from "@deck.gl/layers";
import type { Disposable, TangramApi } from "@open-aviation/tangram-core/api";
import type { MiniskyAircraft } from "./store";
import { ENTITY_TYPE } from ".";

const tangramApi = inject<TangramApi>("tangramApi");
if (!tangramApi) {
  throw new Error("assert: tangram api not provided");
}

interface Trail {
  id: string;
  path: [number, number][];
}

const selectedIds = ref<ReadonlySet<string>>(new Set());
const selectionDisposable = tangramApi.selection.onChanged(map => {
  selectedIds.value = map.get(ENTITY_TYPE) || new Set();
});

const layerDisposable: Ref<Disposable | null> = ref(null);

// Trails come from the core trajectory store, which this plugin feeds over
// api.bus (see index.ts). Reading the reactive refs inside watchEffect
// re-runs this whenever a selected trajectory grows.
const stopWatching = watchEffect(() => {
  if (!tangramApi.map.isReady.value) return;

  const trails: Trail[] = [];
  for (const id of selectedIds.value) {
    const points = tangramApi.trajectory.get({ id, type: ENTITY_TYPE })
      .value as MiniskyAircraft[];
    const path = points
      .filter(p => p.latitude != null && p.longitude != null)
      .map(p => [p.longitude!, p.latitude!] as [number, number]);
    if (path.length > 1) {
      trails.push({ id, path });
    }
  }

  const layer = new PathLayer<Trail>({
    id: "minisky-trail-layer",
    data: trails,
    getPath: d => d.path,
    getColor: [255, 100, 100, 180],
    widthMinPixels: 2,
    jointRounded: true,
    parameters: { cullMode: "none" }
  });

  layerDisposable.value?.dispose();
  layerDisposable.value = tangramApi.map.setLayer(layer, { slot: "live_trails" });
});

onUnmounted(() => {
  stopWatching();
  layerDisposable.value?.dispose();
  selectionDisposable.dispose();
});
</script>
