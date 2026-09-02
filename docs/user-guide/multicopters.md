# Multicopters

The `MULTICOPTER` plugin adds small electric multirotors to minisky.

## Background

A fixed-wing aircraft needs airspeed over its wings to generate lift and typically changes its horizontal flight path by *banking*.

A multicopter, on the other hand, can generate lift at zero forward speed and redirect its total thrust vector. In minisky, this means it can stop and hover, and its body heading can be controlled independently from its horizontal track.

Note that this plugin does not support helicopters.

## Loading the plugin

Add the plugin to your [configuration](configuration.md):

```toml
[plugins.multicopter]
```

It can also be loaded at runtime with the [`PLUGINS LOAD MULTICOPTER` stack command][command.PLUGINS], or from Python with `await runtime.plugins.load("MULTICOPTER")`.

See the [`MulticopterConfig`][minisky_multicopter.config.MulticopterConfig] API reference for the defaults, units and details.

## Performance model

!!! note

    Minisky currently hardcodes OpenAP as the sole aircraft performance model and does not support multiple performance backends. As a workaround, the multicopter plugin adds custom logic on top of OpenAP and selects its replacement implementations on the first simulation pre-update after the plugin loads, and again after a reset. In the future, minisky will support multiple performance backends and fully decouple the multicopter path from OpenAP.

Custom multicopter types can provide their own airframe and electric-performance data; see [`MulticopterTypeTable`][minisky_multicopter.config.MulticopterTypeTable] and [`RotorAirframeSpec`][minisky_multicopter.config.RotorAirframeSpec].

<!-- NOTE(abraham): commands may change a lot in the future so I'm omitting them.

likewise for battery equations, low energy behaviour, controller details.

also we will refactor the openap boundary so i'm not including them here.
 -->
