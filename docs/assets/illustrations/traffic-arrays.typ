#let dark = sys.inputs.at("theme", default: "light") == "dark"

#let text-fg = if dark { oklch(92%, 0, 0deg) } else { oklch(25%, 0, 0deg) }
#let text-muted = if dark { oklch(65%, 0, 0deg) } else { oklch(45%, 0, 0deg) }
#let panel = if dark { oklch(18%, 0.01, 260deg) } else { oklch(98%, 0.005, 260deg) }
#let border = if dark { oklch(30%, 0.01, 260deg) } else { oklch(85%, 0.01, 260deg) }
#let entity-accent = if dark { oklch(75%, 0.16, 250deg) } else { oklch(45%, 0.16, 250deg) }
#let attribute-accent = if dark { oklch(75%, 0.16, 145deg) } else { oklch(45%, 0.16, 145deg) }
#let text-size = 12pt
#let padding-x = 24pt
#let padding-y = 12pt

#let cell = (width: 80pt, height: 24pt)
#let row-gap-y = 6pt
#let index-above-y = 16pt
#let attribute-left-x = 8pt
#let label-left-x = 50pt
#let brace-width = 6pt
#let brace-gap-x = 8pt
#let attribute-note-width = 60pt
#let continuation-gap-x = 8pt
#let continuation-gap-y = 6pt
#let rows = (
  ([callsign], ([KLM204], [BAW17], [CPA8747])),
  ([lat], ([52.31], [51.47], [22.29])),
  ([lon], ([4.76], [-0.46], [114.14])),
  ([altitude], ([12000], [9500], [2000])),
)

#let entity-count = rows.first().last().len()
#let table-width = entity-count * cell.width
#let table-height = rows.len() * cell.height + (rows.len() - 1) * row-gap-y
#let table-left = attribute-note-width + brace-gap-x + brace-width + attribute-left-x + label-left-x
#let diagram-width = table-left + table-width + continuation-gap-x + cell.width / 2
#let diagram-height = index-above-y + table-height + continuation-gap-y + cell.height

#set text(size: text-size, fill: text-fg, font: "Inter")
#set page(width: auto, height: auto, margin: 0pt, fill: none)

#let data-cell(body) = rect(
  width: cell.width,
  height: cell.height,
  inset: 0pt,
  fill: panel,
  stroke: border,
)[#align(center + horizon)[#text(font: "JetBrainsMonoNL NF")[#body]]]

#pad(x: padding-x, y: padding-y)[
  #block(width: diagram-width, height: diagram-height)[
    #place(top + left, dy: index-above-y, box(
      width: attribute-note-width,
      height: table-height,
    )[#align(right + horizon)[#text(fill: attribute-accent)[
      attributes are arrays
    ]]])

    #place(top + left, dx: attribute-note-width + brace-gap-x, dy: index-above-y, box(
      width: brace-width,
      height: table-height,
    )[#align(center + horizon)[
      #text(fill: attribute-accent)[$ stretch(\{, size: #table-height) $]
    ]])

    #for entity-i in range(entity-count) {
      place(top + left, dx: table-left + entity-i * cell.width, box(
        width: cell.width,
      )[#align(center)[#text(fill: entity-accent)[aircraft #entity-i]]])
    }
    #place(
      top + left,
      dx: table-left + table-width + continuation-gap-x,
      box(width: cell.width / 2)[#text(fill: entity-accent)[...]],
    )

    #for (row-i, (name, values)) in rows.enumerate() {
      let y = index-above-y + row-i * (cell.height + row-gap-y)
      place(top + left, dx: table-left - attribute-left-x - label-left-x, dy: y, box(
        width: label-left-x,
        height: cell.height,
      )[#align(right + horizon)[#text(fill: attribute-accent, weight: "bold")[#name]]])
      for (entity-i, value) in values.enumerate() {
        place(
          top + left,
          dx: table-left + entity-i * cell.width,
          dy: y,
          data-cell(value),
        )
      }
    }
    #place(
      top + left,
      dx: table-left - attribute-left-x - label-left-x,
      dy: index-above-y + table-height + continuation-gap-y,
      box(width: label-left-x, height: cell.height)[
        #align(right + horizon)[#text(fill: attribute-accent)[#sym.dots.v]]
      ],
    )
  ]
]
