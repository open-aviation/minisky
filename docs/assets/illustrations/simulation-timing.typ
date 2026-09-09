#let colours-light = (
  colour-text-subtle: oklch(58%, 0, 0deg),
  colour-axis: oklch(80%, 0.01, 260deg),
  colour-step: oklch(40%, 0.16, 250deg),
  colour-wait: oklch(91%, 0.008, 260deg),
)
#let colours-dark = (
  colour-text-subtle: oklch(48%, 0, 0deg),
  colour-axis: oklch(42%, 0.01, 260deg),
  colour-step: oklch(68%, 0.16, 250deg),
  colour-wait: oklch(30%, 0.01, 260deg),
)

#let main(
  style,
  size-text: 11pt,
  p-x: 24pt,
  p-y: 16pt,
  w-label: 170pt,
  gap-x-label: 18pt,
  w-wall-second: 112pt,
  simulation-duration: 3,
  h-legend: 28pt,
  h-axis-label: 15pt,
  gap-y-axis: 10pt,
  gap-y-row: 30pt,
  h-bar: 13pt,
  w-step: 3pt,
  rows: (
    (speed: 1, simdt: 0.5),
    (speed: 1, simdt: 0.1),
    (speed: 2, simdt: 0.5),
    (speed: 8, simdt: 0.5),
  ),
) = {
  let baseline = rows.first()
  let duration-wall = simulation-duration / baseline.speed
  let w-timeline = duration-wall * w-wall-second
  let w-diagram = w-label + w-timeline
  let y-axis = h-legend + h-axis-label
  let y-row-first = y-axis + gap-y-axis
  let y-row-last = y-row-first + (rows.len() - 1) * gap-y-row
  let h-diagram = y-row-last + h-bar
  let h-guide = h-diagram - y-axis

  let legend-item(colour, label) = grid(
    columns: (9pt, auto),
    column-gutter: 5pt,
    align: horizon,
    rect(width: 9pt, height: 5pt, radius: 1pt, fill: colour), text(fill: style.colour-text, size: size-text - 1pt, label),
  )
  let parameter-label(name, value, baseline-value) = {
    let body = [#name = #value]
    if value != baseline-value { strong(body) } else { body }
  }

  set text(size: size-text, fill: style.colour-text, font: style.font-body)

  pad(x: p-x, y: p-y)[
    #block(width: w-diagram, height: h-diagram)[
      #place(top + left, dx: w-label, box(width: w-timeline, height: h-legend)[
        #align(right + horizon)[
          #grid(
            columns: (auto, auto, auto),
            column-gutter: 14pt,
            align: horizon,
            text(fill: style.colour-text, size: size-text - 1pt)[#simulation-duration s simulated],
            legend-item(style.colour-step, [step]),
            legend-item(style.colour-wait, [wait]),
          )
        ]
      ])

      #place(top + left, dy: h-legend, box(
        width: w-label - gap-x-label,
        height: h-axis-label,
      )[#align(right + horizon)[#text(fill: style.colour-text-subtle)[wall-clock time]]])
      #for second in range(int(duration-wall) + 1) {
        let x = w-label + second * w-wall-second
        place(
          top + left,
          dx: x,
          dy: y-axis,
          line(length: h-guide, angle: 90deg, stroke: 0.7pt + style.colour-axis),
        )
        place(
          top + left,
          dx: x - w-wall-second / 2,
          dy: h-legend,
          box(width: w-wall-second, height: h-axis-label)[
            #align(center + horizon)[#text(fill: style.colour-text-subtle)[#second s]]
          ],
        )
      }

      #for (row-i, row) in rows.enumerate() {
        let y = y-row-first + row-i * gap-y-row
        let interval-wall = row.simdt / row.speed
        let w-interval = interval-wall * w-wall-second
        let step-count = int(simulation-duration / row.simdt)
        let speed-label = parameter-label("speed", row.speed, baseline.speed)
        let simdt-label = parameter-label("simdt", row.simdt, baseline.simdt)

        place(top + left, dy: y, box(width: w-label - gap-x-label, height: h-bar)[
          #align(right + horizon)[#text(font: style.font-mono)[#speed-label, #simdt-label]]
        ])

        for i in range(step-count) {
          let x = w-label + i * w-interval
          place(top + left, dx: x, dy: y, grid(
            columns: (w-step, w-interval - w-step),
            rect(width: w-step, height: h-bar, fill: style.colour-step, stroke: none, radius: 1pt),
            rect(
              width: w-interval - w-step,
              height: h-bar,
              fill: style.colour-wait,
              stroke: none,
              radius: 1pt,
            ),
          ))
        }
      }
    ]
  ]
}

#let preview-input = sys.inputs.at("x-preview", default: none)
#if preview-input != none {
  import "render.typ": preview
  preview(main, colours-light, colours-dark, preview-input)
}
