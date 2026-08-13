"""Griffe extension that derives a class's two attribute tables from `__init__`.

Classes document their state in one `Attributes:` section. This extension
splits that section into two titled tables — `Constructor attributes` for the
attributes assigned from an `__init__` parameter, `Internally managed
attributes` for the rest — so the split never has to be written out, or kept in
sync with the signature, by hand.

A constructor attribute nobody documented still gets a row, typed from the
signature, which is more use than the bare heading-plus-signature block
`show_if_no_docstring` renders for it. Attributes that do carry a type of their
own keep it, and `self.traffic = traffic` loses its meaningless `= traffic`
value along the way.

Every tabulated attribute is then dropped from the class members, so an
attribute is documented once: in the table, or — when it carries a docstring of
its own, which a table cell cannot hold — in its own block.
"""

from __future__ import annotations

import ast
from typing import TYPE_CHECKING, Any

from griffe import (
    Attribute,
    Class,
    DocstringAttribute,
    DocstringSectionAttributes,
    DocstringSectionKind,
    ExprName,
    Extension,
    Function,
    Parser,
)

if TYPE_CHECKING:
    from griffe import Docstring, DocstringSection, Inspector, ObjectNode, Visitor

CONSTRUCTOR_TITLE = "Constructor attributes"
INTERNAL_TITLE = "Internally managed attributes"


class AttributeTables(Extension):
    """Rewrite the attribute sections of every class docstring."""

    def on_class_members(
        self,
        *,
        node: ast.AST | ObjectNode,
        cls: Class,
        agent: Visitor | Inspector,
        **kwargs: Any,
    ) -> None:
        """Split `cls`'s attribute table in two and hide the tabulated members."""
        constructor_attributes = _constructor_attributes(cls)
        _adopt_parameter_types(cls, constructor_attributes)
        if cls.docstring is None:
            return

        sections = _parse(cls.docstring)
        documented = _documented_attributes(sections)
        constructor: list[DocstringAttribute] = []
        for name in constructor_attributes:
            row = documented.pop(name, None) or _undocumented_row(cls, name)
            if row is not None:
                constructor.append(row)
        internal = list(documented.values())

        tables = [
            DocstringSectionAttributes(rows, title=title)
            for rows, title in ((constructor, CONSTRUCTOR_TITLE), (internal, INTERNAL_TITLE))
            if rows
        ]
        if not tables:
            return
        cls.docstring.parsed = _with_attribute_tables(sections, tables)

        for row in constructor + internal:
            member = cls.members.get(row.name)
            # An attribute carrying its own docstring keeps its block: dropping
            # it would lose prose the table has no room for.
            if isinstance(member, Attribute) and member.docstring is None:
                cls.del_member(row.name)


def _constructor_attributes(cls: Class) -> dict[str, str]:
    """Map the public attributes assigned from an `__init__` parameter to it.

    Assignments are matched through the parameter name (`self.commands =
    command_stack`) and, failing that, through the attribute name, and come out
    in signature order.
    """
    init = cls.members.get("__init__")
    if not isinstance(init, Function):
        return {}
    positions = {
        parameter.name: position
        for position, parameter in enumerate(init.parameters)
        if parameter.name not in ("self", "cls")
    }
    sources: dict[str, str] = {}
    for attribute in cls.attributes.values():
        if attribute.name.startswith("_"):
            continue
        value = attribute.value
        # `self.commands = command_stack`: a bare parameter name of this `__init__`.
        if isinstance(value, ExprName) and value.parent is init and value.name in positions:
            sources[attribute.name] = value.name
        elif attribute.name in positions:
            sources.setdefault(attribute.name, attribute.name)
    return {name: sources[name] for name in sorted(sources, key=lambda n: positions[sources[n]])}


def _adopt_parameter_types(cls: Class, constructor_attributes: dict[str, str]) -> None:
    """Type unannotated constructor attributes from the `__init__` signature.

    `self.traffic = traffic` carries no annotation of its own, so the docs would
    render it with an empty Type column. Copying the parameter's annotation onto
    the attribute keeps the signature the single source of truth, instead of
    re-declaring the type in the class body to fill that column.
    """
    init = cls.members.get("__init__")
    if not isinstance(init, Function):
        return
    annotations = {parameter.name: parameter.annotation for parameter in init.parameters}
    for name, parameter in constructor_attributes.items():
        attribute = cls.members.get(name)
        if not isinstance(attribute, Attribute):
            continue
        if attribute.annotation is None:
            attribute.annotation = annotations.get(parameter)
        # `= traffic` restates the parameter name, unlike a literal default.
        if isinstance(attribute.value, ExprName) and attribute.value.name == parameter:
            attribute.value = None


def _parse(docstring: Docstring) -> list[DocstringSection]:
    """Parse a docstring, falling back to Google style outside mkdocstrings."""
    if docstring.parser is None:
        return docstring.parse(Parser.google, warnings=False)
    return docstring.parse()


def _documented_attributes(sections: list[DocstringSection]) -> dict[str, DocstringAttribute]:
    """Map every row of the attribute sections by name, in docstring order."""
    documented: dict[str, DocstringAttribute] = {}
    for section in sections:
        if section.kind is DocstringSectionKind.attributes:
            for attribute in section.value:
                documented.setdefault(attribute.name, attribute)
    return documented


def _undocumented_row(cls: Class, name: str) -> DocstringAttribute | None:
    """Build a table row, typed from the signature, for an undocumented attribute."""
    member = cls.members.get(name)
    if not isinstance(member, Attribute) or member.docstring is not None:
        return None
    return DocstringAttribute(name=name, description="", annotation=member.annotation)


def _with_attribute_tables(
    sections: list[DocstringSection], tables: list[DocstringSectionAttributes]
) -> list[DocstringSection]:
    """Replace the attribute sections of a docstring with the derived tables.

    The tables take the place of the first attribute section, or are appended
    when the docstring documented no attribute at all.
    """
    rebuilt: list[DocstringSection] = []
    inserted = False
    for section in sections:
        if section.kind is not DocstringSectionKind.attributes:
            rebuilt.append(section)
        elif not inserted:
            rebuilt.extend(tables)
            inserted = True
    if not inserted:
        rebuilt.extend(tables)
    return rebuilt
