#let colours-light = (
  colour-border: oklch(85%, 0.01, 260deg),
  colour-entity: oklch(45%, 0.16, 250deg),
  colour-attribute: oklch(45%, 0.16, 145deg),
)
#let colours-dark = (
  colour-border: oklch(30%, 0.01, 260deg),
  colour-entity: oklch(75%, 0.16, 250deg),
  colour-attribute: oklch(75%, 0.16, 145deg),
)

#let main(
  style,
  size-text: 12pt,
  p-x: 24pt,
  p-y: 12pt,
  w-cell: 80pt,
  h-cell: 24pt,
  gap-y-row: 6pt,
  y-table: 16pt,
  gap-x-attribute: 8pt,
  w-label: 50pt,
  w-brace: 6pt,
  gap-x-brace: 8pt,
  w-attribute-note: 60pt,
  gap-x-continuation: 8pt,
  gap-y-continuation: 6pt,
  rows: (
    ([callsign], ([KLM204], [BAW17], [CPA8747])),
    ([lat], ([52.31], [51.47], [22.29])),
    ([lon], ([4.76], [-0.46], [114.14])),
    ([altitude], ([12000], [9500], [2000])),
  ),
) = {
  let entity-count = rows.first().last().len()
  let w-table = entity-count * w-cell
  let h-table = rows.len() * h-cell + (rows.len() - 1) * gap-y-row
  let x-table = w-attribute-note + gap-x-brace + w-brace + gap-x-attribute + w-label
  let w-diagram = x-table + w-table + gap-x-continuation + w-cell / 2
  let h-diagram = y-table + h-table + gap-y-continuation + h-cell

  let data-cell(body) = rect(
    width: w-cell,
    height: h-cell,
    inset: 0pt,
    fill: style.colour-panel,
    stroke: style.colour-border,
  )[#align(center + horizon)[#text(font: style.font-mono)[#body]]]

  set text(size: size-text, fill: style.colour-text, font: style.font-body)

  pad(x: p-x, y: p-y)[
    #block(width: w-diagram, height: h-diagram)[
      #place(top + left, dy: y-table, box(
        width: w-attribute-note,
        height: h-table,
      )[#align(right + horizon)[#text(fill: style.colour-attribute)[attributes are arrays]]])

      #place(top + left, dx: w-attribute-note + gap-x-brace, dy: y-table, box(
        width: w-brace,
        height: h-table,
      )[#align(center + horizon)[#text(fill: style.colour-attribute)[$ stretch(\{, size: #h-table) $]]])

      #for entity-i in range(entity-count) {
        place(top + left, dx: x-table + entity-i * w-cell, box(
          width: w-cell,
        )[#align(center)[#text(fill: style.colour-entity)[aircraft #entity-i]]])
      }
      #place(
        top + left,
        dx: x-table + w-table + gap-x-continuation,
        box(width: w-cell / 2)[#text(fill: style.colour-entity)[...]],
      )

      #for (row-i, (name, values)) in rows.enumerate() {
        let y = y-table + row-i * (h-cell + gap-y-row)
        place(top + left, dx: x-table - gap-x-attribute - w-label, dy: y, box(
          width: w-label,
          height: h-cell,
        )[#align(right + horizon)[#text(fill: style.colour-attribute, weight: "bold")[#name]]])
        for (entity-i, value) in values.enumerate() {
          place(
            top + left,
            dx: x-table + entity-i * w-cell,
            dy: y,
            data-cell(value),
          )
        }
      }
      #place(
        top + left,
        dx: x-table - gap-x-attribute - w-label,
        dy: y-table + h-table + gap-y-continuation,
        box(width: w-label, height: h-cell)[
          #align(right + horizon)[#text(fill: style.colour-attribute)[#sym.dots.v]]
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
