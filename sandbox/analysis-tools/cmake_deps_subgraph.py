import argparse
import re

def extract_subgraph(dot_file, target_name, output_file):
    with open(dot_file, "r") as f:
        lines = f.readlines()

    nodes = set()
    edges = []
    node_pattern = re.compile(r'"([^"]+)" \[label="([^"]+)"')
    edge_pattern = re.compile(r'"([^"]+)" -> "([^"]+)"')

    # First pass: find all nodes related to target
    for line in lines:
        if target_name in line:
            match = node_pattern.search(line)
            if match:
                nodes.add(match.group(1))

    print(len(nodes))
    # Second pass: collect edges between relevant nodes
    for line in lines:
        match = edge_pattern.search(line)
        if match and (match.group(1) in nodes or match.group(2) in nodes):
            nodes.add(match.group(1))
            nodes.add(match.group(2))
            edges.append(line)

    # Output subgraph
    with open(output_file, "w") as f:
        f.write("digraph G {\n")
        for line in lines:
            if any(n in line for n in nodes):
                f.write(line)
        for edge in edges:
            f.write(edge)
        f.write("}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract subgraph for a specific CMake target.")
    parser.add_argument("dot_file", help="Path to the full Graphviz .dot file")
    parser.add_argument("target_name", help="CMake target name to extract")
    parser.add_argument("-o", "--output", default="subgraph.dot", help="Output subgraph file")

    args = parser.parse_args()
    extract_subgraph(args.dot_file, args.target_name, args.output)
