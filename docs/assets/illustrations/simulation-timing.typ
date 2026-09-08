#let dark = sys.inputs.at("theme", default: "light") == "dark"

#let text-fg = if dark { oklch(92%, 0, 0deg) } else { oklch(25%, 0, 0deg) }
#let text-muted = if dark { oklch(48%, 0, 0deg) } else { oklch(58%, 0, 0deg) }
#let axis = if dark { oklch(42%, 0.01, 260deg) } else { oklch(80%, 0.01, 260deg) }
#let step-accent = if dark { oklch(68%, 0.16, 250deg) } else { oklch(40%, 0.16, 250deg) }
#let wait-accent = if dark { oklch(30%, 0.01, 260deg) } else { oklch(91%, 0.008, 260deg) }
#let text-size = 11pt
#let padding-x = 24pt
#let padding-y = 16pt

#let label-width = 170pt
#let label-gap-x = 18pt
#let wall-second-width = 112pt
#let simulation-duration = 3
#let legend-height = 28pt
#let axis-label-height = 15pt
#let axis-gap-y = 10pt
#let row-gap-y = 30pt
#let bar-height = 13pt
#let step-width = 3pt
#let rows = (
  (speed: 1, simdt: 0.5),
  (speed: 1, simdt: 0.1),
  (speed: 2, simdt: 0.5),
  (speed: 8, simdt: 0.5),
)
#let baseline = rows.first()

#let wall-duration = simulation-duration / baseline.speed
#let timeline-width = wall-duration * wall-second-width
#let diagram-width = label-width + timeline-width
#let axis-y = legend-height + axis-label-height
#let first-row-y = axis-y + axis-gap-y
#let last-row-y = first-row-y + (rows.len() - 1) * row-gap-y
#let diagram-height = last-row-y + bar-height
#let guide-height = diagram-height - axis-y

#set text(size: text-size, fill: text-fg, font: "Inter")
#set page(width: auto, height: auto, margin: 0pt, fill: none)

#let legend-item(color, label) = grid(
  columns: (9pt, auto),
  column-gutter: 5pt,
  align: horizon,
  rect(width: 9pt, height: 5pt, radius: 1pt, fill: color), text(fill: text-fg, size: text-size - 1pt, label),
)
#let parameter-label(name, value, baseline-value) = {
  let body = [#name = #value]
  if value != baseline-value { strong(body) } else { body }
}
#pad(x: padding-x, y: padding-y)[
  #block(width: diagram-width, height: diagram-height)[
    #place(top + left, dx: label-width, box(width: timeline-width, height: legend-height)[
      #align(right + horizon)[
        #grid(
          columns: (auto, auto, auto),
          column-gutter: 14pt,
          align: horizon,
          text(fill: text-fg, size: text-size - 1pt)[#simulation-duration s simulated],
          legend-item(step-accent, [step]),
          legend-item(wait-accent, [wait]),
        )
      ]
    ])

    #place(top + left, dy: legend-height, box(
      width: label-width - label-gap-x,
      height: axis-label-height,
    )[#align(right + horizon)[#text(fill: text-muted)[wall-clock time]]])
    #for second in range(int(wall-duration) + 1) {
      let x = label-width + second * wall-second-width
      place(
        top + left,
        dx: x,
        dy: axis-y,
        line(length: guide-height, angle: 90deg, stroke: 0.7pt + axis),
      )
      place(
        top + left,
        dx: x - wall-second-width / 2,
        dy: legend-height,
        box(width: wall-second-width, height: axis-label-height)[
          #align(center + horizon)[#text(fill: text-muted)[#second s]]
        ],
      )
    }

    #for (row-i, row) in rows.enumerate() {
      let y = first-row-y + row-i * row-gap-y
      let wall-interval = row.simdt / row.speed
      let interval-width = wall-interval * wall-second-width
      let step-count = int(simulation-duration / row.simdt)
      let speed-label = parameter-label("speed", row.speed, baseline.speed)
      let simdt-label = parameter-label("simdt", row.simdt, baseline.simdt)

      place(top + left, dy: y, box(width: label-width - label-gap-x, height: bar-height)[
        #align(right + horizon)[#text(font: "JetBrainsMonoNL NF")[#speed-label, #simdt-label]]
      ])

      for i in range(step-count) {
        let x = label-width + i * interval-width
        place(top + left, dx: x, dy: y, grid(
          columns: (step-width, interval-width - step-width),
          rect(width: step-width, height: bar-height, fill: step-accent, stroke: none, radius: 1pt),
          rect(width: interval-width - step-width, height: bar-height, fill: wait-accent, stroke: none, radius: 1pt),
        ))
      }
    }
  ]
]
