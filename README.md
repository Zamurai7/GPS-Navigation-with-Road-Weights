# GPS Navigation with Road Weights 🗺️

A sophisticated interactive pathfinding web application that implements advanced algorithms to find optimal routes through a city network. Built with **Shiny for Python** and **Plotly**, this project demonstrates algorithm optimization, interactive visualization, and real-world navigation concepts.

## Features

- **Dual Pathfinding Algorithms**
  - **A* Algorithm**: Heuristic-based pathfinding using Euclidean distance estimation for optimal route finding
  - **BFS (Breadth-First Search)**: Explores all nodes level-by-level to ensure shortest path discovery

- **Interactive Web Interface**
  - Real-time route visualization with color-coded nodes and paths
  - Dropdown selection for starting and destination intersections
  - Live algorithm comparison and performance metrics
  - Responsive design with modern UI/UX

- **Advanced Visualization**
  - Dynamic map with 25 Alexandria city intersections
  - Color-coded nodes: Green (path), Dark Red (explored), Blue (unexplored)
  - Edge labels showing distance (km) and speed limits
  - Real-time graph updates

- **Algorithm Comparison Tab**
  - Side-by-side performance metrics
  - Efficiency improvement percentages
  - Pie chart showing search effort distribution
  - Bar chart comparing nodes explored

- **Route Statistics**
  - Total distance calculation
  - Estimated travel time based on speed limits
  - Nodes explored count
  - Algorithm performance tracking

## How It Works

### Graph Structure
- **25 Nodes**: Representing intersections across Alexandria
- **Bidirectional Edges**: Roads connecting intersections with distance (km) and speed limits (km/hr)
- **Weighted Graph**: Edges contain both distance and speed information for realistic routing

### Pathfinding Logic

**A* Algorithm**
```python
f_score = g_score + heuristic(current, goal)
# g_score: Actual distance from start
# heuristic: Estimated distance to goal (Euclidean)
```

**BFS Algorithm**
```python
# Explores nodes level-by-level using FIFO queue
# Guarantees shortest path discovery
```

## Tech Stack

| Technology | Purpose | Version |
|-----------|---------|---------|
| **Python** | Core language | 3.8+ |
| **Shiny** | Web framework & UI | Latest |
| **Plotly** | Interactive visualizations | 5.0+ |
| **Pandas** | Data manipulation | 1.3+ |

## Installation

1. Clone repository
2. Create virtual environment: `python -m venv venv`
3. Activate: `source venv/bin/activate` (or `venv\Scripts\activate` on Windows)
4. Install: `pip install -r requirements.txt`
5. Run: `python main.py`
6. Open: `http://localhost:8000`

## Key Skills Demonstrated

✅ Algorithm Implementation & Optimization
✅ Graph Theory & Data Structures
✅ Full-Stack Web Development
✅ Data Visualization & Interactive UI
✅ Performance Benchmarking
✅ Clean Code & Documentation

## Resources

- [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Breadth-First Search](https://en.wikipedia.org/wiki/Breadth-first_search)
- [Shiny for Python](https://shiny.posit.co/py/)
- [Plotly Documentation](https://plotly.com/python/)

---
