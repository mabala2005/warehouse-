import streamlit as st
from collections import deque
import heapq
import random
import time



st.set_page_config(
    page_title="Warehouse Robot Path Planning",
    page_icon="🤖",
    layout="wide"
)

ROWS = 10
COLS = 10
START = (0, 0)
END = (9, 9)

DEFAULT_OBSTACLES = {
    (1, 2), (1, 3), (1, 4),
    (2, 4), (3, 4),
    (4, 1), (4, 2), (4, 3),
    (5, 6), (5, 7),
    (6, 2), (6, 3),
    (7, 6), (8, 6)
}



if "obstacles" not in st.session_state:
    st.session_state.obstacles = set(DEFAULT_OBSTACLES)

if "path" not in st.session_state:
    st.session_state.path = []

if "visited" not in st.session_state:
    st.session_state.visited = []

if "result" not in st.session_state:
    st.session_state.result = False

if "message" not in st.session_state:
    st.session_state.message = ""

if "execution_time" not in st.session_state:
    st.session_state.execution_time = 0

if "comparison" not in st.session_state:
    st.session_state.comparison = {}



def neighbors(position):
    row, col = position

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]

    for dr, dc in directions:
        nr = row + dr
        nc = col + dc

        if 0 <= nr < ROWS and 0 <= nc < COLS:
            next_cell = (nr, nc)

            if next_cell not in st.session_state.obstacles:
                yield next_cell


def bfs_shortest():

    queue = deque([START])
    parent = {START: None}
    visited = []

    while queue:

        current = queue.popleft()
        visited.append(current)

        if current == END:
            break

        for nxt in neighbors(current):

            if nxt not in parent:

                parent[nxt] = current
                queue.append(nxt)

    if END not in parent:
        return None, visited

    path = []
    node = END

    while node is not None:

        path.append(node)
        node = parent[node]

    return path[::-1], visited




def heuristic(a, b):
    # Manhattan distance (admissible for 4-direction movement)
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar_shortest():

    counter = 0
    open_heap = [(heuristic(START, END), counter, START)]
    parent = {START: None}
    g_cost = {START: 0}
    closed = set()
    visited = []

    while open_heap:

        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        closed.add(current)
        visited.append(current)

        if current == END:
            break

        for nxt in neighbors(current):

            new_g = g_cost[current] + 1

            if nxt not in g_cost or new_g < g_cost[nxt]:

                g_cost[nxt] = new_g
                parent[nxt] = current
                counter += 1
                f = new_g + heuristic(nxt, END)
                heapq.heappush(open_heap, (f, counter, nxt))

    if END not in parent:
        return None, visited

    path = []
    node = END

    while node is not None:

        path.append(node)
        node = parent[node]

    return path[::-1], visited


def run_algorithm(name):

    start_time = time.perf_counter()

    if name == "A* Search":
        path, visited = astar_shortest()
    else:
        path, visited = bfs_shortest()

    elapsed = time.perf_counter() - start_time

    return path, visited, elapsed


def find_path(algorithm):

    path, visited, elapsed = run_algorithm(algorithm)

    st.session_state.execution_time = elapsed
    st.session_state.visited = visited
    st.session_state.path = path if path else []
    st.session_state.result = True

    # Run both algorithms so that they can be compared
    comparison = {}

    for name in ("BFS", "A* Search"):

        p, v, t = run_algorithm(name)

        comparison[name] = {
            "Path Length (Steps)": len(p) - 1 if p else "No path",
            "Visited Cells": len(v),
            "Execution Time (sec)": f"{t:.6f}"
        }

    st.session_state.comparison = comparison

    if path:
        st.session_state.message = f"Path found successfully using {algorithm}!"
    else:
        st.session_state.message = "No valid path found!"




def reset_warehouse():

    st.session_state.obstacles = set(DEFAULT_OBSTACLES)
    st.session_state.path = []
    st.session_state.visited = []
    st.session_state.result = False
    st.session_state.message = ""
    st.session_state.execution_time = 0
    st.session_state.comparison = {}




def generate_layout():

    cells = [
        (r, c)
        for r in range(ROWS)
        for c in range(COLS)
        if (r, c) != START and (r, c) != END
    ]

    st.session_state.obstacles = set(
        random.sample(cells, 18)
    )

    st.session_state.path = []
    st.session_state.visited = []
    st.session_state.result = False
    st.session_state.message = ""
    st.session_state.execution_time = 0
    st.session_state.comparison = {}




def draw_grid():

    path = st.session_state.path
    visited = st.session_state.visited
    obstacles = st.session_state.obstacles

    html = """
    <div style="
        display:grid;
        grid-template-columns:repeat(10, 1fr);
        gap:5px;
        max-width:650px;
        margin:auto;
        padding:12px;
        background:#202938;
        border-radius:12px;">
    """

    for r in range(ROWS):

        for c in range(COLS):

            cell = (r, c)

            color = "#e5e7eb"
            symbol = ""

            if cell == START:
                color = "#16a34a"
                symbol = "S"

            elif cell == END:
                color = "#dc2626"
                symbol = "E"

            elif cell in obstacles:
                color = "#374151"
                symbol = "X"

            elif cell in path:
                color = "#38bdf8"
                symbol = "●"

            elif cell in visited:
                color = "#fde68a"
                symbol = "·"

            html += f"""
            <div style="
                background:{color};
                height:48px;
                display:flex;
                justify-content:center;
                align-items:center;
                border-radius:5px;
                color:#111827;
                font-size:17px;
                font-weight:bold;">
                {symbol}
            </div>
            """

    html += "</div>"

    st.markdown(html, unsafe_allow_html=True)




def movement_sequence(path):

    if not path:
        return []

    movements = []

    for i in range(1, len(path)):

        r1, c1 = path[i - 1]
        r2, c2 = path[i]

        if r2 > r1:
            movements.append("DOWN")

        elif r2 < r1:
            movements.append("UP")

        elif c2 > c1:
            movements.append("RIGHT")

        elif c2 < c1:
            movements.append("LEFT")

    return movements




st.title("🤖 Warehouse Robot Path Planning")
st.subheader("Shortest Path Finding and Obstacle Avoidance System")

st.write(
    "This application uses Breadth-First Search (BFS) and A* Search "
    "to find the shortest valid route for a warehouse robot."
)

st.divider()

# ---------------- SIDEBAR ----------------

st.sidebar.title("⚙️ Warehouse Controls")

algorithm = st.sidebar.radio(
    "Select Algorithm",
    ["BFS", "A* Search"]
)

st.sidebar.divider()

st.sidebar.write("Manage warehouse obstacles.")

row = st.sidebar.selectbox(
    "Select Row",
    list(range(ROWS))
)

col = st.sidebar.selectbox(
    "Select Column",
    list(range(COLS))
)

selected_cell = (row, col)

if st.sidebar.button("➕ Add Obstacle", use_container_width=True):

    if selected_cell == START or selected_cell == END:
        st.sidebar.warning("Cannot block START or END!")

    else:
        st.session_state.obstacles.add(selected_cell)
        st.session_state.path = []
        st.session_state.visited = []
        st.session_state.result = False
        st.sidebar.success("Obstacle added!")

if st.sidebar.button("➖ Remove Obstacle", use_container_width=True):

    if selected_cell in st.session_state.obstacles:
        st.session_state.obstacles.remove(selected_cell)
        st.session_state.path = []
        st.session_state.visited = []
        st.session_state.result = False
        st.sidebar.success("Obstacle removed!")

    else:
        st.sidebar.warning("No obstacle in this cell!")

st.sidebar.divider()

if st.sidebar.button("🔄 Generate New Layout", use_container_width=True):
    generate_layout()
    st.rerun()

if st.sidebar.button("♻️ Reset Warehouse", use_container_width=True):
    reset_warehouse()
    st.rerun()

st.sidebar.divider()

if algorithm == "BFS":
    st.sidebar.markdown("""
### Algorithm Information

**Algorithm:** BFS

**Data Structure:** Queue (FIFO)

**Movement:** 4 Directions

**Heuristic:** Not Required

**Time Complexity:** O(V + E)

**Space Complexity:** O(V)
""")
else:
    st.sidebar.markdown("""
### Algorithm Information

**Algorithm:** A* Search

**Data Structure:** Priority Queue (heapq)

**Movement:** 4 Directions

**Heuristic:** Manhattan Distance

**Cost Function:** f(n) = g(n) + h(n)

**Time Complexity:** O(E log V)
""")




left, right = st.columns([1.5, 1])

with left:

    st.subheader("🏭 Warehouse Grid")

    st.markdown("""
    🟩 START &nbsp;&nbsp;
    🟥 DESTINATION &nbsp;&nbsp;
    ⬛ OBSTACLE &nbsp;&nbsp;
    🟦 SHORTEST PATH &nbsp;&nbsp;
    🟨 VISITED
    """)

    draw_grid()

with right:

    st.subheader("🚀 Robot Controls")

    st.write(f"Run {algorithm} to find the shortest route.")

    if st.button(
        "🔍 Find Shortest Path",
        use_container_width=True,
        type="primary"
    ):
        find_path(algorithm)

    st.divider()

    st.subheader("📊 Path Information")

    if st.session_state.result:

        if st.session_state.path:

            st.success(st.session_state.message)

            st.metric(
                "Total Distance",
                f"{len(st.session_state.path) - 1} Steps"
            )

            st.metric(
                "Visited Cells",
                len(st.session_state.visited)
            )

            st.metric(
                "Path Cells",
                len(st.session_state.path)
            )

            st.metric(
                "Execution Time",
                f"{st.session_state.execution_time:.6f} sec"
            )

        else:
            st.error(st.session_state.message)

    else:
        st.info("Click Find Shortest Path to start the robot.")



if st.session_state.result and st.session_state.comparison:

    st.divider()

    st.subheader("⚖️ BFS vs A* Comparison")

    st.table(st.session_state.comparison)


if st.session_state.result and st.session_state.path:

    st.divider()

    st.subheader("🧭 Robot Movement Sequence")

    movements = movement_sequence(st.session_state.path)

    st.write(
        f"Total movements: {len(movements)}"
    )

    st.write(" → ".join(movements))

    with st.expander("View Complete Path Coordinates"):

        for i, position in enumerate(st.session_state.path):

            st.write(
                f"Step {i}: Row {position[0]}, Column {position[1]}"
            )



st.divider()

st.markdown(
    "<center>Warehouse Robot Path Planning System | "
    "BFS & A* Algorithms | Python & Streamlit</center>",
    unsafe_allow_html=True
)
