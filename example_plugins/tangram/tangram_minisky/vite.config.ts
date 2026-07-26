import { defineConfig } from "vite";
// Published tangram-core v0.5.0 ships no declaration for the vite-plugin subpath, but a
// linked local checkout does — @ts-expect-error would be "unused" in one of the two modes.
// eslint-disable-next-line @typescript-eslint/ban-ts-comment
// @ts-ignore
import { tangramPlugin } from "@open-aviation/tangram-core/vite-plugin";

export default defineConfig({
  plugins: [tangramPlugin()]
});
