#import "theme.typ" as theme

#let style(theme-name, colours-local) = {
  let colours-shared = if theme-name == "dark" { theme.dark } else { theme.light }
  (
    font-body: theme.font-body,
    font-mono: theme.font-mono,
    ..colours-shared,
    ..colours-local,
  )
}

#let render(main, colours-local, theme-name) = {
  set page(width: auto, height: auto, margin: 0pt, fill: none)
  main(style(theme-name, colours-local))
}

// see: https://myriad-dreamin.github.io/tinymist/feature/preview.html#label-sys.inputs
#let preview(main, colours-light, colours-dark, input) = {
  let args = json(bytes(input))
  let theme-name = args.at("theme", default: "light")
  let colours-local = if theme-name == "dark" { colours-dark } else { colours-light }
  render(main, colours-local, theme-name)
}
