from rich.rule import Rule
from rich.highlighter import ReprHighlighter
from pathlib import Path

import numpy as np
import yourdfpy
from rich.console import Console
from rich.table import Table
from rich.tree import Tree
from rich.text import Text

PACKAGE_DIR = Path(__name__).parent
ROBOT = PACKAGE_DIR / "robot.urdf"
GRIPPER = PACKAGE_DIR / "gripper.urdf"
XLEROBOT = PACKAGE_DIR / "xlerobot.urdf"

console = Console()


def filename_handler(fname: str) -> str:
    if fname.startswith("package://"):
        # Remove package:// prefix and resolve relative to URDF directory
        relative_path = fname.replace("package://", "")
        return str(PACKAGE_DIR / relative_path)
    return fname


def compute_end_links(urdf: yourdfpy.URDF) -> list[str]:
    parents = {j.parent for j in urdf.joint_map.values()}
    children = {j.child for j in urdf.joint_map.values()}
    return sorted(children - parents)


def render_urdf(urdf: yourdfpy.URDF) -> None:
    base = urdf.base_link
    tips = compute_end_links(urdf)
    n_dof = urdf.num_dofs

    console.print(f"[bold]Base link:[/bold] {base}")
    console.print(f"[bold]End links:[/bold] {', '.join(tips)}")
    console.print(f"[bold]DoF:[/bold] {n_dof}\n")

    # ---- Joint table ----
    table = Table(title="Joints", show_lines=True)
    table.add_column("Name")
    table.add_column("Type")
    table.add_column("Parent")
    table.add_column("Child")
    table.add_column("Limits (deg)")

    hl = ReprHighlighter()
    for j in reversed(urdf.joint_map.values()):
        limits = "-"
        if j.limit is not None:
            lower, upper = np.round(np.rad2deg([j.limit.lower, j.limit.upper]), 4)
            limits = hl(Text(f"[{lower}, {upper}]"))

        table.add_row(
            j.name,
            j.type,
            j.parent,
            j.child,
            limits,
        )

    console.print(table, highlight=True)

    # ---- Kinematic tree ----
    tree = Tree(f"[bold]{base}[/bold]")

    children_by_parent = {}
    for j in urdf.joint_map.values():
        children_by_parent.setdefault(j.parent, []).append(j)

    def add(node, link):
        for j in children_by_parent.get(link, []):
            child = node.add(f"{j.child} [dim]({j.name})[/dim]")
            add(child, j.child)

    add(tree, base)
    console.print("\n", tree)


def load_urdf(path: Path) -> yourdfpy.URDF:
    console.print(Rule())
    stem, suffix = path.stem, path.suffix
    console.print(
        f"[green]Loading [/green][bold]{stem}[/bold]{suffix}[white]...[/white]"
    )
    urdf = yourdfpy.URDF.load(
        str(path),
        filename_handler=filename_handler,
    )
    return urdf


def main():
    xlerobot = load_urdf(XLEROBOT)
    render_urdf(xlerobot)
    xlerobot.write_xml_file(XLEROBOT.with_stem(f"{XLEROBOT.stem}_gen"))

    robot = load_urdf(ROBOT)
    render_urdf(robot)
    robot.write_xml_file(ROBOT.with_stem(f"{ROBOT.stem}_gen"))

    gripper = load_urdf(GRIPPER)
    render_urdf(gripper)
    gripper.write_xml_file(GRIPPER.with_stem(f"{GRIPPER.stem}_gen"))


if __name__ == "__main__":
    main()
