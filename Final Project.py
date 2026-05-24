# ==================== 1. IMPORTS ====================
from shiny import App, render, ui, reactive # App = main application, render = renders outputs, ui = UI components, reactive = reactive values/effects
import plotly.graph_objects as go # Low-level Plotly module for creating charts and graphs
import plotly.express as px # High-level Plotly module
import plotly.io as pio # Plotly I/O utilities to convert figures to HTML format for web display
from collections import deque # Double-ended queue "used in bfs"
import heapq # Heap queue module for priority queue operations "used in A*"
import math
import pandas as pd # For datatables/ dataframes

# ==================== 2. DATA ====================
# Dictionary storing all 25 locations
# Key = node ID , Value = tuple of (x-coordinate, y-coordinate) "for positioning on the map"
NODES = {
    1: (120, 100), 2: (280, 80), 3: (450, 110), 4: (620, 150), 5: (800, 120),
    6: (120, 280), 7: (280, 300), 8: (450, 320), 9: (620, 300), 10: (800, 280),
    11: (120, 480), 12: (280, 500), 13: (450, 520), 14: (620, 500), 15: (800, 480),
    16: (180, 640), 17: (380, 670), 18: (600, 630), 19: (820, 620), 20: (80, 380),
    21: (850, 380), 22: (220, 150), 23: (680, 420), 24: (350, 420), 25: (520, 230)
}

# Dictionary storing the labels that the user will choose from
# Key = node ID, Value = location name
NODE_LABELS = {
    1: "Mohamed farid", 2: "Wingat Street", 3: "Main Street", 4: "El montaza", 5: "Stanly Bridge",
    6: "El hadra Elgdeda", 7: "El rasafa", 8: "shobra", 9: "El haram Street", 10: "Gleem beach",
    11: "El zohoor", 12: "El Eza3a bakoos", 13: "Safia Zaglol", 14: "Smouha", 15: "San Stefano",
    16: "Flaming", 17: "Sidi gaber street", 18: "Roshdy tram station", 19: "Foud Street", 20: "El sha3rawe Street",
    21: "Bolkaly tram station", 22: "Kafr abdo", 23: "FCDS Collage", 24: "Al ittihad club",
    25: "El Shatby tram station"
}

# Dictionary storing all bidirectional roads between intersections
# Key = tuple (node1, node2), Value = distance in km
EDGES = {
    # Top row connections (1-5)
    (1, 2): (2.5, 50), (2, 3): (2.0, 45), (3, 4): (2.5, 55), (4, 5): (2.0, 50),

    # Vertical connections from top to middle
    (1, 6): (3.0, 40), (2, 7): (3.5, 35), (3, 8): (3.0, 45), (4, 9): (3.5, 50), (5, 10): (3.0, 48),

    # Middle row connections (6-10)
    (6, 7): (2.5, 50), (7, 8): (2.0, 45), (8, 9): (2.5, 55), (9, 10): (2.0, 50),

    # Vertical connections from middle to bottom
    (6, 11): (3.0, 40), (7, 12): (3.5, 35), (8, 13): (3.0, 45), (9, 14): (3.5, 50), (10, 15): (3.0, 48),

    # Bottom row connections (11-15)
    (11, 12): (2.5, 50), (12, 13): (2.0, 45), (13, 14): (2.5, 55), (14, 15): (2.0, 50),

    # Bottom extensions (16-19)
    (11, 16): (2.8, 40), (12, 17): (3.2, 35), (13, 18): (2.9, 45), (14, 19): (3.1, 50),
    (16, 17): (2.5, 45), (17, 18): (2.3, 40), (18, 19): (2.4, 50),

    # Side connections
    (6, 20): (2.6, 50), (10, 21): (2.8, 48),

    # Extra connections
    (1, 22): (2.3, 45), (2, 22): (1.9, 50), (22, 3): (2.2, 48),
    (3, 25): (2.7, 45), (4, 25): (2.1, 50), (25, 5): (2.5, 48),
    (7, 24): (2.4, 40), (8, 24): (2.2, 45), (24, 9): (2.3, 50),
    (20, 24): (3.0, 35), (23, 25): (2.5, 45), (9, 23): (2.8, 48),
    (14, 23): (3.2, 42), (15, 21): (3.1, 50),

    # Cross connections
    (5, 9): (3.5, 55), (4, 14): (3.8, 48), (12, 18): (3.3, 40),
    (16, 20): (3.1, 35), (20, 11): (2.9, 42), (19, 21): (2.9, 48)
}

# To turn all our nodes into a Graph
# Dictionary storing another Dictionaries, Key = node ID, Value = another dict of {neighbor_node: distance between them}
GRAPH = {node: {} for node in NODES}
for (n1, n2), (distance, speed) in EDGES.items(): # n1 = first node, n2 = second node, distance = road distance in km , speed = speed limits
    GRAPH[n1][n2] = (distance, speed) # Adds forward direction: from n1 to n2 with the distance and speed
    GRAPH[n2][n1] = (distance, speed) # Adds reverse direction: from n2 to n1 (bidirectional road)
    # and we made a reverse direction so if we go from A to B we can go back from B to A

# ==================== 3. PATHFINDING ALGORITHMS ====================

def heuristic(node1, node2): # Function to calculate Euclidean straight-line distance between two nodes
    # node1 = starting node ID, node2 = goal node ID
    x1, y1 = NODES[node1] # x and y coordinates of the starting node
    x2, y2 = NODES[node2] # x and y coordinates of the goal node
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def a_star(start, goal):
    open_set = [(0, start)] # Priority queue storing (f_score, node_id); starts with start node
    came_from = {} # Dictionary to track parent of each node for path reconstruction ### حتى زيادة عن السكشن
    g_score = {node: float('inf') for node in NODES} # Dictionary storing actual distance from start to each node; initialized to infinity
    g_score[start] = 0 # Set the distance from start itself is 0
    explored = [] # List tracking order of nodes explored

    while open_set:
        # _, : Makes me take only the node_id bec, I don't need the f_score
        # heapq.heappop(): Pops node with lowest f_score from priority queue
        _, current = heapq.heappop(open_set)

        # Marks node as explored only if not already explored
        if current not in explored:
            explored.append(current)

        # If we reached the destination
        if current == goal:
            path = [goal] # Start a path with goal node
            while goal in came_from: # Trace back through parent nodes
                goal = came_from[goal]
                path.append(goal) # Add parent to path
            return list(reversed(path)), explored # Return path in correct order (start→goal) and all explored nodes

        # For each neighbor of current node
        # neighbor = neighbor node ID, cost = distance to that neighbor
        for neighbor, (distance, speed) in GRAPH[current].items():
            alternative_g = g_score[current] + distance # Calculate distance to reach this neighbor through current node
            # alternative_g: Total distance start to this neighbor if we take this specific path
            # g_score[current]: How much effort it took to get to where you are now
            # distance: How much more effort it takes to reach the neighbor

            # If this path to neighbor is better than previous path
            if alternative_g < g_score[neighbor]:
                came_from[neighbor] = current # Update parent of neighbor to current node
                g_score[neighbor] = alternative_g # Update g_score (actual distance)
                f_score = alternative_g + heuristic(neighbor, goal) # Calculate f_score = actual distance + estimated remaining distance
                heapq.heappush(open_set, (f_score, neighbor)) # Add neighbor to priority queue with its f_score

    return [], explored # Return empty path if no route found, and all explored nodes


def bfs(start, goal):
    queue = deque([start]) # Initialize queue with start node (FIFO - First In First Out)
    came_from = {start: None} # Dictionary to track parent of each node; start has no parent
    explored = [start] # Dictionary to track parent of each node; start has no parent

    while queue: # while queue is not empty
        current = queue.popleft() # Remove and get first node from queue (FIFO)

        # If we reached the destination
        if current == goal:
            path = [] # Initialize empty path
            while current is not None: # Trace back through parents until we reach start (current = None)
                path.append(current) # Add current node to path
                current = came_from[current] # Move to parent node
            return list(reversed(path)), explored # Return path in correct order (start→goal) and all explored nodes

        for neighbor in GRAPH[current]: # For each neighbor of current node
            if neighbor not in came_from: # If neighbor hasn't been visited yet
                came_from[neighbor] = current # Set current as parent of neighbor
                queue.append(neighbor) # Add neighbor to end of queue
                explored.append(neighbor) # Mark neighbor as explored

    return [], explored # Return empty path if no route found, and all explored nodes

# Function to sum up all distances along a path
# path = list of node IDs from start to goal
def calculate_path_distance(path):
    if len(path) < 2: # If path has less than 2 nodes (empty or single node)
        return 0 # Return zero distance

    total = 0 # Initialize distance counter
    for i in range(len(path) - 1): # Loop through each consecutive pair of nodes in path
        distance, speed = GRAPH[path[i]][path[i + 1]] # unpack distance only to be used
        total += distance # Add the distance between 2 consecutive nodes

    return round(total, 2) # Return total distance rounded to 2 decimal places


# ==================== 4. MAP VISUALIZATION ====================
# Function to create interactive map showing "path" and "explored nodes"
def create_map_figure(explored_nodes, path):
    fig = go.Figure() # Create empty Plotly figure object

    # Loop through each edge (road) in the city
    for (n1, n2), (distance, speed) in EDGES.items():
        x1, y1 = NODES[n1] # x,y coordinates of first node
        x2, y2 = NODES[n2] # x,y coordinates of first node

        # Check if this edge is in the final path (forward direction)
        is_in_path = (n1, n2) in [(path[i], path[i + 1]) for i in range(len(path) - 1)] if len(path) > 1 else False
        # Also check reverse direction (n2,n1) since roads are bidirectional
        is_in_path = is_in_path or ((n2, n1) in [(path[i], path[i + 1]) for i in range(len(path) - 1)] if len(path) > 1 else False)

        line_color = "#10B981" if is_in_path else "#D1D5DB" # Green (#10B981) if edge is in path, gray (#D1D5DB) if not
        line_width = 5 if is_in_path else 2 # Thicker line (5) for path edges, thinner line (2) for other edges

        # Add a line trace (visual line) to the figure
        fig.add_trace( # "Hey Figure (fig), add a new layer of data to yourself."
            go.Scatter( # Drawing tool
                x=[x1, x2], y=[y1, y2], # x-coordinates: [start_x, end_x] for the line , y-coordinates: [start_y, end_y] for the line
                mode='lines', # Display mode: 'lines' means draw as line
                line=dict(color=line_color, width=line_width), # Sets line properties: color and width "we did up line 158"
                hoverinfo='text', # When mouse hovers over line show custom text
                text=f"Distance: {distance} km, {speed} km/hr", # The custom text we show
                showlegend=False # TRUST me you want this False :I
            )
        )

        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2 # Calculate midpoint of the line for label placement
        fig.add_annotation( # Add text label to the map
            x=mid_x, y=mid_y,
            text=f"{distance}km, {speed} km/hr", # Show line's distance and speed limit
            font=dict(size=8, color="#000000"),
            showarrow=False # Don't show arrow pointing to label
        )

    # Initialize empty lists to store node properties for visualization
    node_x, node_y, node_color, node_text = [], [], [], []

    # Loop through each node and its coordinates
    for node_id, (x, y) in NODES.items():
        node_x.append(x) # Add x coordinate to node_x list
        node_y.append(y) # Add y coordinate to node_y list
        node_text.append(str(node_id)) # Add node ID as text label

        # Set color based on node status
        if node_id in path:
            color = "#34D399"  # Green: node is in final path
        elif node_id in explored_nodes:
            color = "#800020"  # Dark red: node was explored but not in final path
        else:
            color = "#818CF8"  # Blue: node was never explored

        node_color.append(color) # Add color to node_color list

    fig.add_trace(go.Scatter( # All explained from line 162
        x=node_x,
        y=node_y,
        mode='markers+text', # Display mode: show both markers (dots) and text (labels)
        marker=dict(size=25, color=node_color),
        text=node_text,
        textposition="middle center",
        textfont=dict(size=10, color="white"),
        showlegend=False)
    )

    # Update overall figure layout and appearance
    fig.update_layout(
        margin=dict(b=0, l=0, r=0, t=0), # If we wanted to add any margins to any side "of the 4" of the map
        height=500,
        xaxis=dict(showgrid=False, zeroline=False, showticklabels=False), # Hide grid/zero lines & axis numbers
        yaxis=dict(showgrid=False, zeroline=False, showticklabels=False)
    )

    return fig

# ==================== 5. UI ====================
app_ui = ui.page_fluid(
    ui.head_content(
        ui.tags.link(href="https://fonts.googleapis.com/css2?family=Pacifico&display=swap", rel="stylesheet"),
        ui.tags.style("""
            body { 
                background-color: #F3F4F6; 
                color: #1F2937; 
            }

            .pacifico-text { 
                font-family: 'Pacifico', cursive; 
                text-align: center; 
                font-size: 2.5em; 
                color: #374151;
                font-weight: bold;
                margin-top: 30px;
                margin-bottom: 10px;
            }

            .subtitle { 
                text-align: center; 
                font-size: 1.1em; 
                color: #6B7280; 
                margin-bottom: 30px; 
            }

            .card-container { 
                background-color: #FFFFFF; 
                padding: 20px; 
                border-radius: 12px; 
                border: 1px solid #D1D5DB; 
                margin-bottom: 0px;
                box-shadow: 0 2px 8px rgba(0,0,0,0.08); 
                display: flex;
                flex-direction: column;
            }

            .stat-box { 
                background-color: #F9FAFB; 
                padding: 15px; 
                border-radius: 8px; 
                text-align: center; 
                border-left: 4px solid #1F2937; 
                margin-bottom: 15px; 
            }

            .stat-label { 
                color: #6B7280; 
                font-size: 0.85em; 
                margin-bottom: 5px; 
            }

            .stat-value { 
                color: #374151; 
                font-size: 1.6em; 
                font-weight: bold; 
            }

            .plot-container { 
                background-color: white; 
                padding: 15px; 
                border-radius: 10px; 
                margin-bottom: 0px;
                box-shadow: 0 4px 6px rgba(0,0,0,0.1); 
                display: flex;
                align-items: center;
                justify-content: center;
                flex: 1;
            }

            .nav-tabs { 
                justify-content: center !important; 
                display: flex !important; 
            }

            table.dataTable { 
                width: 100% !important; 
                table-layout: fixed !important; 
            }

            table.dataTable th,
            table.dataTable td { 
                font-size: 10px !important; 
                padding: 6px !important; 
                white-space: normal !important; 
                word-wrap: break-word; 
            }

            .shiny-input-container {
                color: #1F2937;
            }

            .btn-primary {
                background-color: #374151 !important;
                border-color: #374151 !important;
            }

            .btn-primary:hover {
                background-color: #1F2937 !important;
                border-color: #1F2937 !important;
            }

            .btn-secondary {
                background-color: #6B7280 !important;
                border-color: #6B7280 !important;
            }

            .btn-secondary:hover {
                background-color: #4B5563 !important;
                border-color: #4B5563 !important;
            }

            .route-found {
                background-color: #D1F4E0; 
                padding: 15px; 
                border-radius: 8px; 
                margin-top: 0px; 
                border-left: 4px solid #10B981;
            }

            .route-waiting {
                background-color: #FEF3C7; 
                padding: 15px; 
                border-radius: 8px; 
                margin-top: 0px; 
                border-left: 4px solid #F59E0B;
            }

            select {
                color: #1F2937 !important;
            }

            .form-check-label {
                color: #1F2937 !important;
            }

            #map_plot {
                width: 100% !important;
                height: 100% !important;
            }

            #comparison_chart {
                width: 100% !important;
                height: 400px !important;
            }
        """)
    ),

    ui.h1("GPS Navigation with Road Weights 🗺️", class_="pacifico-text"), # Title for the website, Class: applying the CSS styling & bec, in python keyword: class_
    ui.p("Find the optimal route using advanced pathfinding algorithms", class_="subtitle"), # Subtitle and it's class

    ui.layout_sidebar( # Create layout with sidebar on left and main content on right
        ui.sidebar( # Create the sidebar panel
            ui.div( # create HTML div container "to write HTML in python file"
                ui.h3("Navigation Control", style="text-align: center;"),
                ui.hr(), # تعمل line فاصل بينهم

                ui.p(ui.strong("From Intersection:")), # ui.p() = paragraph , ui.strong() = bold text inside paragraph
                ui.input_select("from_node", "", # make a dropdown select input list (id, label, choices: from NODE_LABELS, default to the 1st option)
                                choices={str(i): f"{i}: {NODE_LABELS[i]}" for i in sorted(NODES.keys())}, selected="1"), # sorted(): sorts keys by numerical order

                ui.p(ui.strong("To Intersection:")), # same shit >:I
                ui.input_select("to_node", "", choices={str(i): f"{i}: {NODE_LABELS[i]}" for i in sorted(NODES.keys())},
                                selected="5"),

                ui.p(ui.strong("Algorithm:")),
                ui.input_radio_buttons("algorithm", "", # Create radio button group, (id, choices: A*/bfs, default to A* "bec, it's BETTERRRR")
                                       choices={"A*": "A* Algorithm", "BFS": "BFS Algorithm"}, selected="A*"),

                ui.hr(),
                # Create clickable button, "btn" = button styling / "btn-primary" = primary button color (dark gray) / "w-100" = width 100% (full width)
                ui.input_action_button("run_search", "START SEARCH", class_="btn btn-primary w-100"),
                ui.br(), ui.br(), # Leaves space between them like in System.out.println("") in java
                ui.input_action_button("reset_btn", "RESET", class_="btn btn-secondary w-100"), # same SHITTT >:{{ but "btn-secondary"

                class_="card-container" # See all this HTML section, we will put it in a container using class: "card-container"
            ),
            width="500px" # Set the width of the sidebar
        ),

        ui.div( # New HTML section
            ui.navset_tab( # Make Tabs
                ui.nav_panel("Map", # Frist panel "tab" (title, row layout)
                             ui.br(),
                             ui.row( # ui.output_ui() = output area for dynamic UI/HTML / "map_plot" = ID of output to display here
                                 ui.column(7, ui.output_ui("map_plot")), # width: 7 = column takes 7/12 of row width (58%)

                                 ui.column(5,
                                           ui.div(
                                               ui.h4("Nodes Explored", style="text-align: center;"),
                                               ui.br(),
                                               ui.output_ui("nodes_explored_list"),
                                               style="background-color: white; padding: 15px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); height: 100%;"
                                           )
                                           )
                             ),
                             ui.br(),
                             ui.div(
                                 ui.h4("Path Found"),
                                 ui.output_ui("route_info"),
                                 style="background-color: white; padding: 15px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);"
                             )
                             ),

                ui.nav_panel("Algorithm Comparison",
                             ui.br(),
                             ui.row(
                                 ui.column(6,
                                           ui.div(ui.output_table("comparison_table"), class_="plot-container"), # Comparison Table
                                           ui.br(),
                                           ui.div(ui.output_ui("efficiency_pie"), class_="plot-container") # Efficiency Pie
                                           ),
                                 ui.column(6, ui.div(ui.output_ui("comparison_chart"), class_="plot-container")) # Comparison Chart
                             )
                             )
            ),
            class_="card-container", # Put all the Main Panel in a container
            #style="flex: 1; overflow: auto; display: flex; flex-direction: column;" # Style it
            # flex: 1 = take remaining space
            # overflow: auto = add scrollbars if content too large
            # display: flex = flexbox layout
            # flex-direction: column = stack items vertically

        )
    ),

    ui.hr(),
    ui.h3("Route Statistics"), # Heading for statistics section

    ui.row( # Create a row for stat boxes
        ui.column(3,
                  ui.div(
                      ui.p("Total Distance", class_="stat-label"),
                      ui.p(ui.output_text("stat_distance"), class_="stat-value"),
                      class_="stat-box"
                  )
                  ),
        ui.column(3,
                  ui.div(
                      ui.p("Est. Travel Time", class_="stat-label"),
                      ui.p(ui.output_text("stat_time"), class_="stat-value"),
                      class_="stat-box"
                  )
                  ),
        ui.column(3,
                  ui.div(
                      ui.p("Nodes Explored", class_="stat-label"),
                      ui.p(ui.output_text("stat_explored"), class_="stat-value"),
                      class_="stat-box"
                  )
                  ),
        ui.column(3,
                  ui.div(
                      ui.p("Algorithm Used", class_="stat-label"),
                      ui.p(ui.output_text("stat_algo"), class_="stat-value"),
                      class_="stat-box"
                  )
                  )
    )
)


# ==================== 6. SERVER ====================
def server(input, output, session):
    path_result = reactive.Value({"path": [], "explored": [], "algo": ""}) # algo: algorithm name used "A* / BFS"
    # reactive.Value() = reactive variable that triggers updates when changed

    @reactive.Effect # Decorator that creates a reactive effect (runs when inputs change)
    def _(): # Function name is _ (underscore) because we don't call it directly
        if input.run_search() == 0: # If "START SEARCH" button has NOT been clicked yet / input.run_search() = number of times button clicked
            path_result.set({"path": [], "explored": [], "algo": ""})
            return

        from_node = int(input.from_node()) # Get starting node from dropdown, convert to integer # ID: line 404
        to_node = int(input.to_node()) # Get destination node from dropdown, convert to integer # ID: line 408
        algo = input.algorithm() # Get selected algorithm (A* or BFS) # ID: line 412

        if algo == "A*": # If the algorithm chosen is A* apply the A* function
            path, explored = a_star(from_node, to_node)
        else: # If not then apply the BFS function
            path, explored = bfs(from_node, to_node)

        # Update reactive value with results
        path_result.set({
            "path": path,
            "explored": explored,
            "algo": algo,
            "from": from_node,
            "to": to_node
        })

    @reactive.Effect
    def _(): # Another reactive effect for reset button
        if input.reset_btn() > 0:
            path_result.set({"path": [], "explored": [], "algo": ""}) # Clear all results

    @output # Register as output
    @render.ui # Render as UI (HTML)
    def map_plot(): # Render function for map visualization
        result = path_result.get() # Get current path_result value
        fig = create_map_figure(result.get("explored", []), result.get("path", []))
        # Create map figure with explored nodes and path
        # .get() with default value [] if key doesn't exist

        return ui.HTML(pio.to_html(fig, include_plotlyjs='cdn', div_id="map_plot"))
        # Convert figure to HTML and return for display
        # include_plotlyjs='cdn' = load Plotly from CDN instead of locally "for efficiency and memory stuff"
        # div_id = unique ID for the div container

    @output
    @render.ui
    def route_info(): # Render function for route information display
        result = path_result.get()
        path = result.get("path", []) # Get final path

        if len(path) > 0: # If path exists (route found)
            route_text = " → ".join([NODE_LABELS.get(n, str(n)) for n in path]) # Make a String Path: A → B → C...
            return ui.div(
                ui.h5(f"✅ Route Found: {route_text}"),
                class_="route-found"
            )
        else: # If no path (waiting or no route)
            return ui.div(
                ui.p("🔍 Select 'From' and 'To' intersections, then click 'START SEARCH' to find a route"),
                class_="route-waiting"
            )

    @output
    @render.text
    def stat_distance():
        result = path_result.get()
        distance = calculate_path_distance(result.get("path", []))
        return f"{distance} km" if distance > 0 else "—"

    @output
    @render.text
    def stat_time():
        result = path_result.get()
        path = result.get("path", [])

        total_time = 0
        for i in range(len(path) - 1):
            distance, speed = GRAPH[path[i]][path[i + 1]]  # ← Get both distance and speed
            # time = distance / speed * 60 (to convert hours to minutes)
            time_segment = (distance / speed) * 60
            total_time += time_segment
        return f"{round(total_time, 1)} min" if total_time > 0 else "—"

    @output
    @render.text
    def stat_explored():
        result = path_result.get()
        return str(len(result.get("explored", [])))

    @output
    @render.text
    def stat_algo():
        result = path_result.get()
        return result.get("algo", "—")

    @output
    @render.ui
    def nodes_explored_list():
        result = path_result.get()
        explored = result.get("explored", [])

        if len(explored) == 0:
            return ui.p("No nodes explored yet", style="color: #9CA3AF;")

        nodes_html = "".join([
            f'<div style="padding: 8px; margin: 5px 0; background-color: #93C5FD; border-radius: 5px; font-weight: bold;">{i + 1}. Node {node} - {NODE_LABELS.get(node, "Unknown")}</div>'
            for i, node in enumerate(explored)
        ])

        return ui.HTML(f'<div style="max-height: 500px; overflow-y: auto;">{nodes_html}</div>')

    @output
    @render.table
    def comparison_table():
        result = path_result.get()

        # Check if we have valid from and to nodes
        start = result.get("from")
        goal = result.get("to")

        if start is None or goal is None:
            # Return empty table if no search has been run
            return pd.DataFrame({"Status": ["No search run yet"]})

        # Run BOTH algorithms
        path_a_star, explored_a_star = a_star(start, goal)
        path_bfs, explored_bfs = bfs(start, goal)

        # Calculate distances
        distance_a_star = calculate_path_distance(path_a_star)
        distance_bfs = calculate_path_distance(path_bfs)

        # Calculate how much more efficient A* is than BFS
        if len(explored_bfs) > 0:
            efficiency_improvement = round((1 - len(explored_a_star) / len(explored_bfs)) * 100, 1)
        else:
            efficiency_improvement = 0

        # Build comparison table
        return pd.DataFrame({
            "Algorithm": ["A*", "BFS"],
            "Distance (km)": [f"{distance_a_star} km", f"{distance_bfs} km"],
            "Nodes Explored": [len(explored_a_star), len(explored_bfs)],
            "Efficiency": [f"{efficiency_improvement}% more efficient", "Baseline"]
        })

    @output
    @render.ui
    def efficiency_pie():
        result = path_result.get()
        start = result.get("from")
        goal = result.get("to")

        if start is None or goal is None:
            return ui.div()

        # Run both algorithms with REAL data
        _, explored_a_star = a_star(start, goal)
        _, explored_bfs = bfs(start, goal)

        pie_data = pd.DataFrame({
            "Algorithm": ["A*", "BFS"],
            "Nodes Explored": [len(explored_a_star), len(explored_bfs)]
        })

        fig = px.pie(
            pie_data,
            values='Nodes Explored',
            names='Algorithm',
            title="Search Effort Distribution",
            color_discrete_sequence=["#374151", "#6B7280"],
            hole=0.4
        )

        fig.update_layout(
            margin=dict(t=40, b=20, l=20, r=20),
            plot_bgcolor='#F9FAFB',
            paper_bgcolor='#F9FAFB',
            height=350,
            showlegend=True
        )

        return ui.HTML(pio.to_html(fig, include_plotlyjs='cdn', div_id="efficiency_pie"))

    @output
    @render.ui
    def comparison_chart():
        result = path_result.get()
        start = result.get("from")
        goal = result.get("to")

        if start is None or goal is None:
            return ui.div()

        # Run both algorithms with REAL data
        _, explored_a_star = a_star(start, goal)
        _, explored_bfs = bfs(start, goal)

        fig = px.bar(
            x=["A*", "BFS"],
            y=[len(explored_a_star), len(explored_bfs)],
            labels={"x": "Algorithm", "y": "Nodes Explored"},
            color=["#374151", "#374151"],
            text=[len(explored_a_star), len(explored_bfs)],
            title="Nodes Explored Comparison"
        )

        fig.update_layout(
            showlegend=False,
            plot_bgcolor='#F9FAFB',
            paper_bgcolor='#F9FAFB',
            height=500,
            xaxis_title="Algorithm",
            yaxis_title="Nodes Explored"
        )

        return ui.HTML(pio.to_html(fig, include_plotlyjs='cdn', div_id="comparison_chart"))


# ==================== 7. RUN APP ====================
app = App(app_ui, server)

if __name__ == "__main__":
    app.run()