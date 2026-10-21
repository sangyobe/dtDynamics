#!/usr/bin/env python3
"""Fail-closed content checks for current dtDynamics v0.1.3 documentation."""

from pathlib import Path
from html import unescape
import hashlib
import json
import re
import shutil
import subprocess
import sys

from guide_provenance import discover_local_guide_assets

ROOT = Path(__file__).resolve().parents[1]
failures = []


def require(condition, message):
    if not condition:
        failures.append(message)


def text(rel):
    path = ROOT / rel
    require(path.is_file(), f"missing documentation file: {rel}")
    return path.read_text() if path.is_file() else ""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


targets = [
    "README.md",
    "docs/manual/modeling.md",
    "docs/manual/integration.md",
    "docs/manual/example.md",
    "docs/manual/limitations.md",
    "docs/manual/benchmarks.md",
    "docs/manual/validation.md",
    "docs/manual/release.md",
    "docs/manual.md",
    "docs/release-notes/v0.1.3.md",
]
contents = {path: text(path) for path in targets}
for path, body in contents.items():
    require(re.search(r"[가-힣]", body) is not None, f"current narrative is not Korean: {path}")

# Versioned guide Markdown sources, render inputs, and PDFs are tracked release
# artifacts. Each guide's provenance manifest binds the rendered PDF to every
# source asset it consumes so a stale binary cannot pass content checks.
guide_source_paths = (
    "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md",
    "docs/guides/usage_guide/dtdynamics-usage-migration-guide.md",
)
guide_sources = {
    rel: (ROOT / rel).read_text()
    for rel in guide_source_paths
    if (ROOT / rel).is_file()
}
for path, body in guide_sources.items():
    require(re.search(r"[가-힣]", body) is not None,
            f"current guide narrative is not Korean: {path}")


def validate_pdf_provenance(label, manifest_rel, artifacts, expected):
    manifest_path = ROOT / manifest_rel
    source_path = artifacts["source_sha256"]
    assets, discovery_errors = discover_local_guide_assets(source_path, ROOT / "docs")
    for error in discovery_errors:
        require(False, f"{label} PDF provenance {error}")
    for field, path in artifacts.items():
        require(path.is_file(),
                f"{label} PDF provenance input is missing: {path.relative_to(ROOT)}")
    for rel, path in assets.items():
        require(path.is_file(),
                f"{label} PDF provenance asset is missing: {path.relative_to(ROOT)}")
    require(manifest_path.is_file(),
            f"{label} PDF requires a tracked source/build provenance manifest")
    if not (manifest_path.is_file() and
            all(path.is_file() for path in artifacts.values()) and
            all(path.is_file() for path in assets.values())):
        return
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        require(False, f"{label} PDF provenance manifest is invalid JSON: {error}")
        return
    for field, value in expected.items():
        require(manifest.get(field) == value,
                f"{label} PDF provenance manifest has wrong {field}: {manifest.get(field)!r}")
    for field, path in artifacts.items():
        require(manifest.get(field) == sha256(path),
                f"{label} PDF provenance manifest hash is stale for {path.relative_to(ROOT)}")
    manifest_assets = manifest.get("assets")
    require(isinstance(manifest_assets, dict),
            f"{label} PDF provenance manifest has no asset hash map")
    if isinstance(manifest_assets, dict):
        require(set(manifest_assets) == set(assets),
                f"{label} PDF provenance manifest asset set is incomplete or unexpected")
        for rel, path in assets.items():
            require(manifest_assets.get(rel) == sha256(path),
                    f"{label} PDF provenance asset hash is stale for {path.relative_to(ROOT)}")


validate_pdf_provenance(
    "model-guide",
    "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.provenance.json",
    {
        "source_sha256": ROOT / "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md",
        "theme_sha256": ROOT / "docs/guides/themes/hmg.css",
        "pdf_sha256": ROOT / "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf",
    },
    {
        "schema": 2,
        "renderer": "@marp-team/marp-cli@4.5.1",
        "browser": "google-chrome-stable",
        "source": "mjcf-xml-dtdynspec-dtdynmodel-guide.md",
        "theme": "../themes/hmg.css",
        "pdf": "mjcf-xml-dtdynspec-dtdynmodel-guide.pdf",
    },
)

validate_pdf_provenance(
    "usage-guide",
    "docs/guides/usage_guide/dtdynamics-usage-migration-guide.provenance.json",
    {
        "source_sha256": ROOT / "docs/guides/usage_guide/dtdynamics-usage-migration-guide.md",
        "theme_sha256": ROOT / "docs/guides/themes/hmg.css",
        "pdf_sha256": ROOT / "docs/guides/usage_guide/dtdynamics-usage-migration-guide.pdf",
    },
    {
        "schema": 2,
        "renderer": "@marp-team/marp-cli@4.5.1",
        "browser": "google-chrome-stable",
        "source": "dtdynamics-usage-migration-guide.md",
        "theme": "../themes/hmg.css",
        "pdf": "dtdynamics-usage-migration-guide.pdf",
    },
)

readme = contents["README.md"]
require("find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)" in readme,
        "README must require the exact installed v0.1.3 package")
require("docs/release-notes/v0.1.3.md" in readme, "README must identify the current v0.1.3 release note")

workflow = "\n".join(contents[path] for path in (
    "docs/manual/modeling.md", "docs/manual/integration.md", "docs/manual/example.md"
))
for token in (
    "LoadXmlFile", "DecodeXml", "EncodeXml", "SaveXmlFile", "ValidateSpec",
    "CompileSpecChecked", "CompileSpec", "Resize", "BodyId", "JointId", "SiteId",
    "ActuatorId", "DTDYN_INVALID_INDEX", "IsCompatible",
):
    require(token in workflow, f"manual workflow is missing {token}")
require("dimension" in workflow.lower() or "차원" in workflow,
        "IsCompatible documentation must explain its dimension-only contract")
require("neutral" in workflow.lower() or "중립" in workflow,
        "manual must record the neutral/default-state initializer NO-GO")
modeling = contents["docs/manual/modeling.md"]
for lookup in ("body", "joint", "site", "actuator"):
    require(f"{lookup} == DTDYN_INVALID_INDEX" in modeling,
            f"modeling lookup example does not check {lookup} sentinel")
api_manual = text("docs/manual/api.md")
for token in ("UpdateSpec", "fixed-layout", "exclusive", "false", "stale"):
    require(token in api_manual,
            f"public API map must document the v0.1.3 UpdateSpec contract: {token}")

release = contents["docs/release-notes/v0.1.3.md"]
for token in ("0.1.3", "UpdateSpec", "dtArray", "Polyspace", "raw defect 0"):
    require(token in release, f"v0.1.3 release note is missing {token}")
static_analysis = contents["docs/manual/validation.md"]
for token in ("Polyspace", "MISRA C++:2023", "Justified", "File does not compile",
              "polyspace/checkers.xml", "Bug Finder 237", "179개 rule",
              "tools/test_polyspace_checker_profile.py", "ci/polyspace.sh",
              "tools/polyspace_result_gate.py", "parser exit code"):
    require(token in static_analysis or token in contents["docs/manual.md"],
            f"static-analysis manuals must state the implemented release gate: {token}")
for path in ("README.md", "docs/manual/validation.md", "docs/manual.md"):
    body = contents[path]
    require("Bug Finder 237개" in body and "MISRA C++:2023 179개 rule" in body,
            f"{path} must state the exact locked Polyspace checker profile")
    require("ci/polyspace.sh" in body and "tools/polyspace_result_gate.py" in body,
            f"{path} must state the actual Polyspace result-gate path")
for path in ("README.md", "docs/manual/validation.md", "docs/manual.md", "docs/manual/release.md"):
    body = contents[path]
    require("canonical schema:" in body and "`ID`부터 `Col`" in body and "16개 column" in body,
            f"{path} must document the exact canonical Polyspace TSV schema")
polyspace_ci = text("ci/polyspace.sh")
for token in ('rm -f "${REPORT}"', 'test -s "${REPORT}"',
              'python3 tools/polyspace_result_gate.py --input "${REPORT}"',
              "-protocol https"):
    require(token in polyspace_ci,
            f"Polyspace CI must fail closed and use HTTPS transport: {token}")
polyspace_gate = text("tools/polyspace_result_gate.py")
require("CANONICAL_COLUMNS" in polyspace_gate and
        "headers != list(CANONICAL_COLUMNS)" in polyspace_gate,
        "Polyspace gate must reject every noncanonical TSV header/schema")
require(re.search(r"-protocol\s+http(?:\s|\\\\|$)", polyspace_ci) is None,
        "Polyspace CI must not transmit Access credentials over plaintext HTTP")
require("REVIEWED_STATUSES=" not in polyspace_ci and "awk -F'\\t'" not in polyspace_ci,
        "Polyspace CI must use the canonical parser rather than a divergent hand-rolled gate")
ci_config = text(".gitlab-ci.yml")
require("polyspace-job:" in ci_config and "- bash ci/polyspace.sh" in ci_config and
        "- reports/polyspace/" in ci_config,
        "GitLab analysis job must execute the strict gate and retain its canonical artifact")

install_validator = text("tools/validate_install_export.sh")
require("ReadTextFile" not in install_validator,
        "installed consumer must not reimplement filesystem XML loading")
require("dtDynSpec::LoadXmlFile" in install_validator,
        "installed consumer path branch must exercise package-owned LoadXmlFile")
require("CONSUMER_FILE_BUILD_DIR" in install_validator and
        "install-consumer-model.xml" in install_validator and
        '"${CONSUMER_FILE_BUILD_DIR}/dtDynamicsInstallConsumer"' in install_validator and
        "PASS: installed LoadXmlFile consumer executed" in install_validator,
        "default install validation must compile, run, and report a LoadXmlFile consumer")
require("CompileSpecChecked(spec)" in install_validator,
        "installed consumer must exercise checked compilation")
require("ValidateSpec(spec)" in install_validator,
        "installed consumer must exercise public validation")
require("zeroModel.nq != 0 || zeroModel.nv != 0" in install_validator and
        "zeroData.Resize(zeroModel)" in install_validator,
        "installed consumer must exercise zero-DoF resize")
for family in ("BodyId", "JointId", "SiteId", "ActuatorId"):
    require(f"spec.{family}(missing)" in install_validator and
            f"model.{family}(missing)" in install_validator,
            f"installed consumer must exercise {family} missing-name sentinels")
for token in ("INSTALL_LIBDIR", "CMAKE_INSTALL_LIBDIR", "NO_DEFAULT_PATH",
              "DTDYNAMICS_PACKAGE_DIR", "DTDYNAMICS_EXPECTED_PREFIX",
              "DTDYNAMICS_EXPECTED_LIBRARY_DIR",
              "_dtdynamics_require_within_selected_library_dir",
              "IMPORTED_LOCATION_RELEASE", "CONSUMER_RELOCATED_BUILD_DIR"):
    require(token in install_validator,
            f"install/export validator must enforce staged package provenance: {token}")
require('mv "${INSTALL_PREFIX}" "${RELOCATED_PREFIX}"' in install_validator,
        "install/export validator must relocate the staged prefix before rerunning a consumer")
package_validator = text("tools/validate_package.sh")
require("DTDYNAMICS_INSTALL_LIBDIRS:-lib lib64" in package_validator and
        "DTDYNAMICS_INSTALL_LIBDIR=\"${install_libdir}\"" in package_validator,
        "package validation must exercise both lib and lib64 install layouts by default")
release_packager = text("tools/package_release.sh")
for token in ('release_tag="v${VERSION}"', "refs/tags/${release_tag}^{commit}",
              "git get-tar-commit-id", "${tag_commit}", "does not match project version"):
    require(token in release_packager,
            f"release archive script must enforce exact tag/archive identity: {token}")
release_manual = contents["docs/manual/release.md"]
for token in ("lib`·`lib64", "NO_DEFAULT_PATH", "${prefix}/${install_libdir}",
              "provenance.json", "embedded commit ID", "exact `v0.1.3` tag"):
    require(token in release_manual,
            f"release manual must document staged layout and archive identity gates: {token}")
require("archive embedded commit ID" in contents["docs/manual.md"] and
        "exact `v0.1.3` tag" in contents["docs/manual.md"],
        "top-level release procedure must document v-prefixed archive tag identity")

for directory in (ROOT / "include/dtDynamics", ROOT / "src", ROOT / "benchmark"):
    for path in sorted(directory.rglob("*")):
        if not path.is_file() or path.suffix not in {".h", ".cpp"}:
            continue
        for match in re.finditer(r"\\version\s+(\S+)", path.read_text()):
            require(match.group(1) == "0.1.3",
                    f"nonhistorical public-header version is stale: {path.relative_to(ROOT)}={match.group(1)}")

model_guide = guide_sources.get(guide_source_paths[0])
if model_guide is not None:
    require("find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)" in model_guide,
            "model guide must require the exact v0.1.3 installed package")
    require("CompileSpecChecked" in model_guide, "model guide must teach checked compilation")
    require("DTDYN_INVALID_INDEX" in model_guide, "model guide must check lookup sentinels")
    require("0.1.3" in model_guide, "model guide must identify package 0.1.3")
    require("EditBody" in model_guide and "immutable collection" in model_guide,
            "model guide must document callback-scoped immutable collection edits")
    for stale_mutation in ("spec.Bodies()[", "spec.Joints()[", "spec.Sites()[", "spec.Actuators()["):
        require(stale_mutation not in model_guide,
                f"model guide retains mutable collection example: {stale_mutation}")
    require("fixed topology; `UpdateSpec` only" in model_guide,
            "model guide must qualify dtDynModel as fixed topology with API-controlled physical updates")
    for token in ("UpdateSpec", "fixed-layout", "exclusive model access",
                  "false without model writes", "recompute stale outputs"):
        require(token in model_guide,
                f"model guide must document the UpdateSpec contract: {token}")
    require("Every spec edit requires checked compilation" not in model_guide,
            "model guide retains a universal compile/resize claim")
    require("ignored by native dynamics" not in model_guide,
            "model guide still claims unsupported XML elements are silently ignored")
    require("dtDyn::" not in model_guide,
            "model guide current examples still use a nonexistent dtDyn namespace")
    require("ForwardKinematics(model, data);" in model_guide,
            "model guide site-Jacobian edit example must stage kinematics first")
    require("audit_release_readiness.sh" in model_guide and
            "not shipped by this package" in model_guide.lower(),
            "model guide must separate package gates from external LeoQuad gates")
    require("no allocation or XML parsing inside the real-time tick" not in model_guide and
            "pre-sized `dtDynData` and caller-owned outputs on the documented benchmark path" in model_guide and
            "no XML parsing in the tick; zero counted C++ `new` is limited to the benchmark counter scope, not a whole-process or malloc-family heap guarantee" in model_guide,
            "model guide still makes an unscoped realtime allocation claim")
usage_guide = guide_sources.get(guide_source_paths[1])
if usage_guide is not None:
    require("find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)" in usage_guide,
            "usage guide must require exact package 0.1.3")
    require("Each cell is `median / p95`" in usage_guide,
            "usage guide reference-facing table must use median/p95")
    require("5.751 / 5.881" in usage_guide and "17.962 / 18.090" in usage_guide,
            "usage guide quadruped snapshot is not promoted from the accepted v0.1.3 CSV")
    require("`LoadXmlFile` → `CompileSpecChecked` → `dtDynData::Resize`" in usage_guide,
            "usage migration mapping must teach package-owned load and checked compilation")
    require("`LoadXmlFile` / `DecodeXml` → `CompileSpecChecked` → `Resize`" in usage_guide and
            "native `DecodeXml` → `CompileSpec`" not in usage_guide,
            "usage migration table must use the fail-closed checked model-loading path")
    require(usage_guide.count("dtDyn::") == 1 and
            "dtDyn::RobotEngineBase" in usage_guide,
            "usage guide may retain dtDyn:: only in the labeled legacy engine excerpt")
    require("audit_release_readiness.sh" in usage_guide and
            "not shipped here" in usage_guide.lower(),
            "usage guide must separate package gates from external LeoQuad gates")
    require("No XML parsing, no allocation" not in usage_guide and
            "nothing allocated or parsed" not in usage_guide,
            "usage guide still makes an unscoped allocation-free claim")
    require("ValidateSpec(spec" in usage_guide,
            "usage guide must validate an externally edited spec before UpdateSpec")
    for token in ("UpdateSpec", "fixed-layout physical update", "exclusive model access",
                  "false: no model writes", "recompute stale outputs"):
        require(token in usage_guide,
                f"usage guide must document the UpdateSpec runtime-update contract: {token}")
    for stale_mutation in ("spec.Bodies()[", "spec.Joints()[", "spec.Sites()[", "spec.Actuators()["):
        require(stale_mutation not in usage_guide,
                f"usage guide retains mutable collection example: {stale_mutation}")

for name in ("dtdynamics_to_be.svg",):
    canonical = ROOT / "docs/figures" / name
    for rel in (f"docs/guides/model_guide/figures/{name}",
                f"docs/guides/usage_guide/figures/{name}"):
        copy = ROOT / rel
        require(canonical.is_file() and copy.is_file() and canonical.read_bytes() == copy.read_bytes(),
                f"guide figure is stale relative to canonical docs figure: {rel}")
canonical_as_is = ROOT / "docs/figures/dtdynamics_as_is.svg"
usage_as_is = ROOT / "docs/guides/usage_guide/figures/dtdynamics_as_is.svg"
require(canonical_as_is.is_file() and usage_as_is.is_file() and
        canonical_as_is.read_bytes() == usage_as_is.read_bytes(),
        "usage guide As-Is figure is stale relative to canonical docs figure")

model_layers_dot = text("docs/manual/diagrams/model_layers.dot")
model_layers_svg = unescape(text("docs/manual/diagrams/model_layers.svg"))
for body, rel in ((model_layers_dot, "docs/manual/diagrams/model_layers.dot"),
                  (model_layers_svg, "docs/manual/diagrams/model_layers.svg")):
    require("CompileSpecChecked" in body and "ValidateSpec" in body,
            f"{rel} must distinguish checked validation from raw CompileSpec")
    require("UpdateSpec" in body and "fixed-layout" in body,
            f"{rel} must show the fixed-layout UpdateSpec path")
    require("dtDynData results stale" in body and "recompute before consumption" in body,
            f"{rel} must send successful UpdateSpec output staleness to existing dtDynData")
    require("physical values mutable via UpdateSpec" in body and
            "success: fixed-layout physical write" in body,
            f"{rel} must distinguish UpdateSpec's successful model write from its false no-write path")
    require("rebuild caches" in body and "reader-free safe swap" in body,
            f"{rel} must show structural candidate cache rebuild and reader-free safe swap")
    require("CompileSpec\\nvalidate + assign addresses" not in body,
            f"{rel} falsely attributes validation to raw CompileSpec")

lifecycle_svg = unescape(text("docs/guides/model_guide/figures/mjcf_dtdyn_lifecycle.svg"))
for token in ("CompileSpecChecked", "UpdateSpec", "fixed-layout physical update",
              "exclusive model access", "false: no model writes", "recompute stale outputs",
              "Compile independent candidate", "rebuild caches", "reader-free safe swap"):
    require(token in lifecycle_svg,
            f"model-guide lifecycle SVG is missing UpdateSpec contract token: {token}")
for stale_claim in ("Spec changes recompile", "Model changes resize data"):
    require(stale_claim not in lifecycle_svg,
            f"model-guide lifecycle SVG retains universal stale claim: {stale_claim}")
fixed_layout_validation = re.search(
    r'<g aria-label="fixed-layout validation">(?P<body>.*?)</g>', lifecycle_svg, re.DOTALL
)
update_boundary_index = lifecycle_svg.find('aria-label="UpdateSpec control-safe boundary"')
require(
    fixed_layout_validation is not None and
    "ValidateSpec" in fixed_layout_validation.group("body") and
    'aria-label="validated fixed-layout spec to UpdateSpec"' in lifecycle_svg and
    fixed_layout_validation.start() < update_boundary_index,
    "model-guide lifecycle SVG must validate editable fixed-layout input before UpdateSpec",
)

pdftotext = shutil.which("pdftotext")
require(pdftotext is not None, "pdftotext is required for guide PDF content validation")
if pdftotext:
    pdf_requirements = {
        "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf":
            ("CompileSpecChecked", "UpdateSpec", "fixed-layout physical update",
             "exclusive model access", "false: no model writes", "recompute stale outputs",
             "DTDYN_INVALID_INDEX", "ForwardKinematics", "dt::Dynamics",
             "find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)",
             "pre-sized dtDynData and caller-owned outputs on the documented benchmark path",
             "no XML parsing in the tick; zero counted C++ new is limited to the benchmark counter scope, not a whole-process",
             "or malloc-family heap guarantee",
             "0.1.3"),
        "docs/guides/usage_guide/dtdynamics-usage-migration-guide.pdf":
            ("find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)",
             "0.1.3", "dt::Dynamics", "p95", "5.751", "17.962",
             "UpdateSpec", "ValidateSpec", "fixed-layout physical update", "exclusive model access",
             "false: no model writes", "recompute stale outputs"),
    }
    for rel, tokens in pdf_requirements.items():
        path = ROOT / rel
        require(path.is_file() and path.stat().st_size > 0, f"missing/empty guide PDF: {rel}")
        if path.is_file():
            result = subprocess.run([pdftotext, str(path), "-"], check=False, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            require(result.returncode == 0, f"pdftotext failed for {rel}: {result.stderr.strip()}")
            for token in tokens:
                require(token in result.stdout, f"guide PDF {rel} is stale or missing {token}")

pdfinfo = shutil.which("pdfinfo")
pdffonts = shutil.which("pdffonts")
require(pdfinfo is not None, "pdfinfo is required for guide PDF structure validation")
require(pdffonts is not None, "pdffonts is required for guide font validation")
if pdfinfo and pdffonts:
    guide_pairs = (
        ("docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md",
         "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.pdf", 49),
        ("docs/guides/usage_guide/dtdynamics-usage-migration-guide.md",
         "docs/guides/usage_guide/dtdynamics-usage-migration-guide.pdf", 39),
    )
    for source_rel, pdf_rel, release_pages in guide_pairs:
        source_path, pdf_path = ROOT / source_rel, ROOT / pdf_rel
        info = subprocess.run([pdfinfo, str(pdf_path)], check=False, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        require(info.returncode == 0, f"pdfinfo failed for {pdf_rel}: {info.stderr.strip()}")
        if info.returncode == 0:
            pages = next((int(line.split(":", 1)[1]) for line in info.stdout.splitlines()
                          if line.startswith("Pages:")), -1)
            require(pages == release_pages,
                    f"v0.1.3 guide PDF page count changed: {pdf_rel}={pages}, expected={release_pages}")
            if source_path.is_file():
                source_pages = sum(line.strip() == "---" for line in source_path.read_text().splitlines()) - 1
                require(pages == source_pages,
                        f"guide source/PDF slide mismatch: {source_rel}={source_pages}, {pdf_rel}={pages}")
            require(any("960 x 540 pts" in line for line in info.stdout.splitlines()
                        if line.startswith("Page size:")),
                    f"guide PDF is not 16:9 960x540 points: {pdf_rel}")
            title = next((line.split(":", 1)[1].strip() for line in info.stdout.splitlines()
                          if line.startswith("Title:")), "")
            require("0.1.3" in title and "0.1.2" not in title,
                    f"guide PDF metadata title is stale: {pdf_rel}: {title}")
        fonts = subprocess.run([pdffonts, str(pdf_path)], check=False, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        require(fonts.returncode == 0, f"pdffonts failed for {pdf_rel}: {fonts.stderr.strip()}")
        rows = fonts.stdout.splitlines()[2:] if fonts.returncode == 0 else []
        require(bool(rows), f"guide PDF has no reported fonts: {pdf_rel}")
        for row in rows:
            columns = row.split()
            # Font names can contain spaces. The embedded/subset/Unicode flags are
            # positioned relative to the trailing object and generation columns.
            require(len(columns) >= 7 and columns[-5] == "yes" and columns[-3] == "yes",
                    f"guide PDF font is not embedded/Unicode-capable: {pdf_rel}: {row}")

if failures:
    print("FAIL: documentation content validation", file=sys.stderr)
    for failure in failures:
        print(f" - {failure}", file=sys.stderr)
    raise SystemExit(1)
print("PASS: current v0.1.3 documentation content validated")
