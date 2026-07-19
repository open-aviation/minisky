import { defineConfig } from "vite";
// @ts-expect-error -- tangram-core v0.5.0 publishes no declaration for the vite-plugin subpath
import { tangramPlugin } from "@open-aviation/tangram-core/vite-plugin";

export default defineConfig({
  plugins: [tangramPlugin()]
});
