<template>
  <div
    class="minisky-count"
    :title="title"
  >
    <IconButton
      class="plane-icon"
      :title="title"
    >
      <SvgIcon :path="FLIGHT_ICON" />
    </IconButton>
    <span class="count">{{ miniskyStore.siminfo?.ntraf ?? 0 }}</span>
    <span
      class="state"
      :class="stateClass"
    >
      {{ miniskyStore.connected ? (miniskyStore.siminfo?.state_name ?? "—") : "OFF" }}
    </span>
    <span
      v-if="miniskyStore.connected && miniskyStore.siminfo"
      class="simt"
    >
      {{ simTime }}
    </span>
  </div>
</template>

<script setup lang="ts">
import { computed } from "vue";
import { IconButton, SvgIcon } from "@open-aviation/tangram-core/components";
import { miniskyStore } from "./store";

// tangram-core's ICON_PATHS has no plane icon, so inline the Material Symbols
// Rounded "flight" path (viewBox matches MATERIAL_ICON_VIEW_BOX "0 -960 960 960").
const FLIGHT_ICON =
  "M400-408 147-307q-24 10-45.5-4.5T80-352v-22q0-12 5.5-23t15.5-18l299-209v-176q0-33 23.5-56.5T480-880q33 0 56.5 23.5T560-800v176l299 209q10 7 15.5 18t5.5 23v22q0 26-21.5 40.5T813-307L560-408v144l103 72q8 6 12.5 14.5T680-159v24q0 20-16.5 32.5T627-96l-147-44-147 44q-20 6-36.5-6.5T280-135v-24q0-10 4.5-18.5T297-192l103-72v-144Z";

const simTime = computed(() => {
  const t = Math.floor(miniskyStore.siminfo?.simt ?? 0);
  const h = Math.floor(t / 3600);
  const m = Math.floor((t % 3600) / 60);
  const s = t % 60;
  const pad = (n: number) => n.toString().padStart(2, "0");
  return `${pad(h)}:${pad(m)}:${pad(s)}`;
});

const stateClass = computed(() => {
  if (!miniskyStore.connected) return "off";
  switch (miniskyStore.siminfo?.state_name) {
    case "OP":
      return "op";
    case "HOLD":
      return "hold";
    default:
      return "init";
  }
});

const title = computed(() =>
  miniskyStore.connected
    ? `MiniSky · ${miniskyStore.siminfo?.scenname || "no scenario"} · ${
        miniskyStore.siminfo?.speed ?? 1
      }x`
    : "MiniSky simulator not connected"
);
</script>

<style scoped>
.minisky-count {
  display: flex;
  align-items: center;
  gap: 0.35rem;
  font-size: 10pt;
  color: var(--t-fg);
  white-space: nowrap;
}

.plane-icon {
  color: var(--t-accent2);
  padding: 0;
  min-width: 0;
  min-height: 0;
}

.plane-icon :deep(svg) {
  fill: var(--t-accent2);
}

.count {
  font-weight: bold;
}

.simt {
  color: var(--t-muted);
  font-variant-numeric: tabular-nums;
}

.state {
  font-size: 8pt;
  font-weight: bold;
  padding: 1px 5px;
  border-radius: 8px;
}

.state.op {
  background: #1c7c2e;
  color: #ffffff;
}

.state.hold {
  background: #b58900;
  color: #ffffff;
}

.state.init {
  background: var(--t-border);
  color: var(--t-fg);
}

.state.off {
  background: #7c1c1c;
  color: #ffffff;
}
</style>
