#let colour-trajectory-light = oklch(50%, 0.05, 155deg)
#let colour-intent-light = oklch(50%, 0.05, 65deg)
#let colours-light = (
  colour-text-muted: oklch(45%, 0, 0deg),
  colour-trajectory: colour-trajectory-light,
  colour-intent: colour-intent-light,
  colour-trajectory-panel: colour-trajectory-light.lighten(99%),
  colour-intent-panel: colour-intent-light.lighten(99%),
)
#let colour-trajectory-dark = oklch(75%, 0.03, 155deg)
#let colour-intent-dark = oklch(75%, 0.05, 65deg)
#let colours-dark = (
  colour-text-muted: oklch(65%, 0, 0deg),
  colour-trajectory: colour-trajectory-dark,
  colour-intent: colour-intent-dark,
  colour-trajectory-panel: colour-trajectory-dark.darken(70%),
  colour-intent-panel: colour-intent-dark.darken(70%),
)

#let main(
  style,
  size-text: 10pt,
  leading: 4pt,
  p-x: 20pt,
  p-y: 14pt,
  h-node: 44pt,
  w-reference: 70pt,
  gap-x-reference-intent: 40pt,
  w-intent: 70pt,
  gap-x-intent-simulator: 75pt,
  w-simulator: 70pt,
  gap-x-simulator-comparator: 58pt,
  size-comparator: 12pt,
  gap-x-comparator-error: 20pt,
  w-error: 90pt,
  gap-y-route-node: 50pt,
  gap-x-label: 4pt,
  gap-y-label: 4pt,
  w-arrow-head: 4pt,
  h-arrow-head: 6pt,
) = {
  let w-comparator-line = size-comparator * 0.45
  let stroke-connector = 1pt + style.colour-text
  let stroke-connector-dotted = (paint: style.colour-text, thickness: 1pt, dash: "dotted")

  // reference -> intent -> simulator -> comparator -> error
  let x-reference = 0pt
  let length-reference-intent = gap-x-reference-intent
  let x-intent = x-reference + w-reference + length-reference-intent
  let length-intent-simulator = gap-x-intent-simulator
  let x-simulator = x-intent + w-intent + length-intent-simulator
  let length-simulator-comparator = gap-x-simulator-comparator
  let x-comparator = x-simulator + w-simulator + length-simulator-comparator
  let length-comparator-error = gap-x-comparator-error
  let x-error = x-comparator + size-comparator + length-comparator-error
  let w-diagram = x-error + w-error

  // reference trajectory -> simulator initial state, comparator
  let y-route-top = 0pt
  let y-main = y-route-top + gap-y-route-node
  let y-centre = y-main + h-node / 2
  let h-diagram = y-main + h-node
  let x-reference-centre = x-reference + w-reference / 2
  let x-simulator-centre = x-simulator + w-simulator / 2
  let x-comparator-centre = x-comparator + size-comparator / 2
  let length-simulator-input = y-main - y-route-top
  let length-comparator-input = y-centre - size-comparator / 2 - y-route-top

  let node(width, body, fill: style.colour-panel) = rect(
    width: width,
    height: h-node,
    inset: 8pt,
    fill: fill,
    stroke: stroke-connector,
    radius: 4pt,
  )[#align(center + horizon)[#body]]

  let arrow-right(length, stroke: stroke-connector) = box(width: length, height: h-arrow-head)[
    #place(
      left + top,
      dy: h-arrow-head / 2,
      line(length: length - w-arrow-head, stroke: stroke),
    )
    #place(
      left + top,
      dx: length - w-arrow-head,
      polygon(
        fill: style.colour-text,
        stroke: none,
        (0pt, 0pt),
        (w-arrow-head, h-arrow-head / 2),
        (0pt, h-arrow-head),
      ),
    )
  ]

  let above-arrow-label(x, width, body) = context {
    let label = box(width: width)[#align(center)[#body]]
    let h-label = measure(label).height
    place(
      top + left,
      dx: x,
      dy: y-centre - h-label - gap-y-label,
      label,
    )
  }

  set text(size: size-text, fill: style.colour-text, font: style.font-body)
  set par(leading: leading)

  pad(x: p-x, y: p-y)[
    #block(width: w-diagram, height: h-diagram)[
      #place(
        top + left,
        dx: x-reference,
        dy: y-main,
        node(
          w-reference,
          stack(
            dir: ttb,
            spacing: 4pt,
            text(fill: style.colour-trajectory)[reference trajectory],
            text(fill: style.colour-trajectory)[$y(t)$],
          ),
          fill: style.colour-trajectory-panel,
        ),
      )

      #place(
        top + left,
        dx: x-reference + w-reference,
        dy: y-centre - h-arrow-head / 2,
        arrow-right(length-reference-intent, stroke: stroke-connector-dotted),
      )
      #above-arrow-label(
        x-reference + w-reference,
        length-reference-intent,
        text(size: size-text - 1pt, fill: style.colour-text-muted)[infer],
      )

      #place(
        top + left,
        dx: x-intent,
        dy: y-main,
        node(
          w-intent,
          text(fill: style.colour-intent)[controller/pilot intent],
          fill: style.colour-intent-panel,
        ),
      )

      #place(
        top + left,
        dx: x-intent + w-intent,
        dy: y-centre - h-arrow-head / 2,
        arrow-right(length-intent-simulator),
      )
      #above-arrow-label(
        x-intent + w-intent,
        length-intent-simulator,
        stack(
          dir: ttb,
          spacing: 4pt,
          text(size: size-text - 1pt, fill: style.colour-intent)[waypoints, vertical and speed profiles],
          text(fill: style.colour-intent)[$hat(u)(t)$],
        ),
      )

      #place(
        top + left,
        dx: x-simulator,
        dy: y-main,
        node(
          w-simulator,
          stack(
            dir: ttb,
            spacing: 4pt,
            [minisky],
            [$dot(x) = f(x,$ #text(fill: style.colour-intent)[$u$]$)$],
          ),
        ),
      )

      #place(
        top + left,
        dx: x-simulator + w-simulator,
        dy: y-centre - h-arrow-head / 2,
        arrow-right(length-simulator-comparator),
      )
      #above-arrow-label(
        x-simulator + w-simulator,
        length-simulator-comparator,
        stack(
          dir: ttb,
          spacing: 4pt,
          text(size: size-text - 1pt, fill: style.colour-trajectory)[simulated trajectory],
          text(fill: style.colour-trajectory)[$hat(y)(t)$],
        ),
      )

      #place(
        top + left,
        dx: x-comparator,
        dy: y-centre - size-comparator / 2,
        circle(
          radius: size-comparator / 2,
          fill: style.colour-panel,
          stroke: stroke-connector,
        ),
      )
      #place(
        top + left,
        dx: x-comparator-centre - w-comparator-line / 2,
        dy: y-centre,
        line(length: w-comparator-line, stroke: stroke-connector),
      )

      #place(
        top + left,
        dx: x-comparator + size-comparator,
        dy: y-centre - h-arrow-head / 2,
        arrow-right(length-comparator-error),
      )

      #place(
        top + left,
        dx: x-error,
        dy: y-main,
        node(
          w-error,
          stack(
            dir: ttb,
            spacing: 4pt,
            [reconstruction error],
            [$e($#text(fill: style.colour-trajectory)[$y$]$,$#text(fill: style.colour-trajectory)[ $hat(y)$]$)$],
          ),
        ),
      )

      // top
      #place(
        top + left,
        dx: x-reference-centre,
        dy: y-route-top,
        line(length: x-comparator-centre - x-reference-centre, stroke: stroke-connector),
      )
      #place(
        top + left,
        dx: x-reference-centre,
        dy: y-route-top,
        line(length: y-main - y-route-top, angle: 90deg, stroke: stroke-connector),
      )

      #place(
        top + left,
        dx: x-simulator-centre - h-arrow-head / 2,
        dy: y-route-top,
        box(width: h-arrow-head, height: length-simulator-input)[
          #place(
            left + top,
            dx: h-arrow-head / 2,
            line(
              length: length-simulator-input - w-arrow-head,
              angle: 90deg,
              stroke: stroke-connector-dotted,
            ),
          )
          #place(
            left + top,
            dy: length-simulator-input - w-arrow-head,
            polygon(
              fill: style.colour-text,
              stroke: none,
              (0pt, 0pt),
              (h-arrow-head, 0pt),
              (h-arrow-head / 2, w-arrow-head),
            ),
          )
        ],
      )
      #context {
        let label = text(size: size-text - 1pt, fill: style.colour-text-muted)[initial state]
        place(
          top + left,
          dx: x-simulator-centre + gap-x-label,
          dy: y-route-top + length-simulator-input / 2 - measure(label).height / 2,
          label,
        )
      }

      #place(
        top + left,
        dx: x-comparator-centre - h-arrow-head / 2,
        dy: y-route-top,
        box(width: h-arrow-head, height: length-comparator-input)[
          #place(
            left + top,
            dx: h-arrow-head / 2,
            line(
              length: length-comparator-input - w-arrow-head,
              angle: 90deg,
              stroke: stroke-connector,
            ),
          )
          #place(
            left + top,
            dy: length-comparator-input - w-arrow-head,
            polygon(
              fill: style.colour-text,
              stroke: none,
              (0pt, 0pt),
              (h-arrow-head, 0pt),
              (h-arrow-head / 2, w-arrow-head),
            ),
          )
        ],
      )
    ]
  ]
}

#let preview-input = sys.inputs.at("x-preview", default: none)
#if preview-input != none {
  import "render.typ": preview
  preview(main, colours-light, colours-dark, preview-input)
}
