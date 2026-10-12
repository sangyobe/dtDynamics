#!/usr/bin/env python3
"""Discover rendered guide assets from Markdown and HTML image references."""

from __future__ import annotations

from html.parser import HTMLParser
from pathlib import Path, PurePosixPath
import re
from urllib.parse import urlsplit


_REFERENCE_DEFINITION = re.compile(
    r"(?m)^[ \t]{0,3}\[(?P<label>(?:\\.|[^\]\\])*)\]:[ \t]*(?P<body>.*)$"
)
_HTML_IMG_START = re.compile(r"<\s*img(?=[\s/>])", re.IGNORECASE)
_FENCE_START = re.compile(r"^[ \t]{0,3}(?P<fence>`{3,}|~{3,})")
_MASKED_RENDER_SYNTAX = re.compile(
    r"!\[|<\s*(?:img|picture|source|svg|image|object|embed|iframe|video|audio|track|canvas|style|link|script)\b",
    re.IGNORECASE,
)
_UNSUPPORTED_VISUAL_TAGS = {
    "audio", "canvas", "embed", "iframe", "image", "link", "object", "picture",
    "script", "source", "style", "svg", "track", "video",
}
_RESOURCE_ATTRIBUTES = {"background", "poster", "src", "srcset", "xlink:href"}
_STYLE_URL = re.compile(r"url\s*\(", re.IGNORECASE)


def _blank_non_newline(characters: list[str], start: int, end: int) -> None:
    for index in range(start, end):
        if characters[index] not in {"\r", "\n"}:
            characters[index] = " "


def _mask_non_rendered_markdown(source: str) -> str:
    """Mask fenced/inline code and comments so examples are not render inputs."""
    characters = list(source)
    offset = 0
    active_fence: tuple[str, int] | None = None
    for line in source.splitlines(keepends=True):
        content_end = offset + len(line.rstrip("\r\n"))
        match = _FENCE_START.match(line)
        if active_fence is not None:
            _blank_non_newline(characters, offset, content_end)
            if match is not None:
                fence = match.group("fence")
                if fence[0] == active_fence[0] and len(fence) >= active_fence[1]:
                    active_fence = None
        elif match is not None:
            fence = match.group("fence")
            active_fence = (fence[0], len(fence))
            _blank_non_newline(characters, offset, content_end)
        offset += len(line)

    masked = "".join(characters)
    index = 0
    while index < len(masked):
        if masked[index] != "`":
            index += 1
            continue
        run_end = index
        while run_end < len(masked) and masked[run_end] == "`":
            run_end += 1
        delimiter = masked[index:run_end]
        close = masked.find(delimiter, run_end)
        if close == -1:
            index = run_end
            continue
        _blank_non_newline(characters, index, close + len(delimiter))
        masked = "".join(characters)
        index = close + len(delimiter)

    for match in re.finditer(r"<!--[\s\S]*?-->", "".join(characters)):
        _blank_non_newline(characters, match.start(), match.end())
    return "".join(characters)


def _masked_render_syntax_errors(source: str, rendered_source: str) -> list[str]:
    """Reject visual syntax hidden by a parser mask rather than silently ignoring it."""
    errors: list[str] = []
    for match in _MASKED_RENDER_SYNTAX.finditer(source):
        start, end = match.span()
        if rendered_source[start:end] != source[start:end]:
            errors.append("rendered image syntax inside masked Markdown is unsupported")
    return errors


def _is_escaped(source: str, index: int) -> bool:
    backslashes = 0
    index -= 1
    while index >= 0 and source[index] == "\\":
        backslashes += 1
        index -= 1
    return backslashes % 2 == 1


def _consume_brackets(source: str, start: int) -> tuple[str | None, int]:
    """Read a balanced Markdown bracket sequence starting at ``[``."""
    depth = 1
    index = start + 1
    while index < len(source):
        character = source[index]
        if character == "\\" and index + 1 < len(source):
            index += 2
            continue
        if character == "[":
            depth += 1
        elif character == "]":
            depth -= 1
            if depth == 0:
                return source[start + 1:index], index + 1
        index += 1
    return None, len(source)


def _consume_parenthesized(source: str, start: int) -> tuple[str | None, int]:
    """Read a balanced Markdown destination/title sequence starting at ``(``."""
    depth = 1
    index = start + 1
    while index < len(source):
        character = source[index]
        if character == "\\" and index + 1 < len(source):
            index += 2
            continue
        if character == "(":
            depth += 1
        elif character == ")":
            depth -= 1
            if depth == 0:
                return source[start + 1:index], index + 1
        index += 1
    return None, len(source)


def _first_destination_token(body: str) -> str | None:
    body = body.strip()
    if not body:
        return None
    if body.startswith("<"):
        index = 1
        while index < len(body):
            if body[index] == "\\" and index + 1 < len(body):
                index += 2
                continue
            if body[index] == ">":
                return body[1:index]
            index += 1
        return None

    depth = 0
    index = 0
    while index < len(body):
        character = body[index]
        if character == "\\" and index + 1 < len(body):
            index += 2
            continue
        if character == "(":
            depth += 1
        elif character == ")" and depth > 0:
            depth -= 1
        elif character.isspace() and depth == 0:
            break
        index += 1
    return body[:index] or None


def _normalize_reference_label(label: str) -> str:
    unescaped = re.sub(r"\\(.)", r"\1", label)
    return " ".join(unescaped.split()).casefold()


def _reference_definitions(source: str) -> dict[str, str | None]:
    definitions: dict[str, str | None] = {}
    for match in _REFERENCE_DEFINITION.finditer(source):
        label = _normalize_reference_label(match.group("label"))
        destination = _first_destination_token(match.group("body"))
        if not label or label in definitions:
            definitions[label] = None
        else:
            definitions[label] = destination
    return definitions


def _markdown_image_references(source: str) -> tuple[list[str], list[str]]:
    definitions = _reference_definitions(source)
    references: list[str] = []
    errors: list[str] = []
    index = 0
    while index < len(source) - 1:
        if source[index:index + 2] != "![" or _is_escaped(source, index):
            index += 1
            continue
        alt, after_alt = _consume_brackets(source, index + 1)
        if alt is None:
            errors.append("malformed Markdown image reference: missing closing ]")
            break
        if after_alt < len(source) and source[after_alt] == "(":
            body, after_image = _consume_parenthesized(source, after_alt)
            if body is None:
                errors.append("malformed Markdown image reference: missing closing )")
                index = after_alt + 1
                continue
            destination = _first_destination_token(body)
            if destination is None:
                errors.append("malformed Markdown image reference: missing destination")
            else:
                references.append(destination)
            index = after_image
            continue

        label = alt
        after_image = after_alt
        if after_alt < len(source) and source[after_alt] == "[":
            explicit_label, after_image = _consume_brackets(source, after_alt)
            if explicit_label is None:
                errors.append("malformed Markdown image reference: missing reference label ]")
                index = after_alt + 1
                continue
            if explicit_label:
                label = explicit_label
        key = _normalize_reference_label(label)
        destination = definitions.get(key)
        if destination is None:
            errors.append(f"unresolved or malformed Markdown image reference: {label!r}")
        else:
            references.append(destination)
        index = after_image
    return references, errors


class _ImageHTMLParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=False)
        self.references: list[str] = []
        self.errors: list[str] = []
        self.image_tags = 0

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._record_tag(tag, attrs)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._record_tag(tag, attrs)

    def _record_tag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        normalized_tag = tag.casefold()
        normalized_attrs = [(name.casefold(), value) for name, value in attrs]
        if normalized_tag in _UNSUPPORTED_VISUAL_TAGS:
            self.errors.append(f"unsupported visual HTML tag <{normalized_tag}>")
            return
        if any(value is not None and _STYLE_URL.search(value) for name, value in normalized_attrs if name == "style"):
            self.errors.append(
                f"unsupported CSS url() resource on <{normalized_tag}>"
            )
            return
        if normalized_tag == "img":
            self.image_tags += 1
            sources = [value for name, value in normalized_attrs if name == "src"]
            if len(sources) != 1 or sources[0] is None or not sources[0].strip():
                self.errors.append("HTML img tag must have exactly one non-empty src attribute")
                return
            if any(name == "srcset" for name, _ in normalized_attrs):
                self.errors.append("HTML img srcset is unsupported; use one local src instead")
                return
            self.references.append(sources[0])
            return
        if any(name in _RESOURCE_ATTRIBUTES for name, _ in normalized_attrs):
            self.errors.append(
                f"unsupported resource-bearing HTML attribute on <{normalized_tag}>"
            )


def _html_image_references(source: str) -> tuple[list[str], list[str]]:
    parser = _ImageHTMLParser()
    parser.feed(source)
    parser.close()
    raw_tags = len(_HTML_IMG_START.findall(source))
    if raw_tags != parser.image_tags:
        parser.errors.append("malformed HTML img tag")
    return parser.references, parser.errors


def _image_references(source: str) -> tuple[list[str], list[str]]:
    rendered_source = _mask_non_rendered_markdown(source)
    masked_errors = _masked_render_syntax_errors(source, rendered_source)
    markdown_references, markdown_errors = _markdown_image_references(rendered_source)
    html_references, html_errors = _html_image_references(rendered_source)
    return markdown_references + html_references, masked_errors + markdown_errors + html_errors


def _normalize_local_reference(reference: str) -> tuple[str | None, str | None]:
    parsed = urlsplit(reference)
    if parsed.scheme or parsed.netloc:
        return None, f"image reference must be a local path: {reference!r}"
    if parsed.query or parsed.fragment:
        return None, f"image reference must not use a query or fragment: {reference!r}"
    if "\\" in reference:
        return None, f"image reference must use a portable POSIX path: {reference!r}"
    path = PurePosixPath(parsed.path)
    if not parsed.path or path.is_absolute():
        return None, f"image reference must be a relative local path: {reference!r}"
    normalized = path.as_posix()
    if normalized in {"", "."}:
        return None, f"image reference must name a file: {reference!r}"
    return normalized, None


def discover_local_guide_assets(source_path: Path, docs_root: Path) -> tuple[dict[str, Path], list[str]]:
    """Return source-relative local image assets and fail-closed discovery errors."""
    try:
        source = source_path.read_text(encoding="utf-8")
    except OSError as error:
        return {}, [f"cannot read guide source {source_path}: {error}"]

    assets: dict[str, Path] = {}
    references, errors = _image_references(source)
    resolved_docs_root = docs_root.resolve()
    for reference in references:
        normalized, error = _normalize_local_reference(reference)
        if error is not None:
            errors.append(error)
            continue
        assert normalized is not None
        asset_path = (source_path.parent / normalized).resolve()
        try:
            asset_path.relative_to(resolved_docs_root)
        except ValueError:
            errors.append(f"image reference escapes guide documentation root: {normalized}")
            continue
        if normalized in assets:
            continue
        assets[normalized] = asset_path
        if not asset_path.is_file():
            errors.append(f"referenced guide image is missing: {normalized}")
    return assets, errors
