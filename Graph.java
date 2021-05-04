import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;
import java.io.*;
import java.util.*;


public class Graph {

  // Keep a fast index to nodes in the map
  private Map<Integer, Vertex> vertexNames;
  Collection<Vertex> vertices;

  /**
   * Construct an empty Graph with a map. The map's key is the name of a vertex
   * and the map's value is the vertex object.
   */
  public Graph() {
    vertexNames = new HashMap<>();
  }

  /**
   * Adds a vertex to the graph. Throws IllegalArgumentException if two vertices
   * with the same name are added.
   * 
   * @param v
   *          (Vertex) vertex to be added to the graph
   */
  public void addVertex(Vertex v) {
    if (vertexNames.containsKey(v.name))
      throw new IllegalArgumentException("Cannot create new vertex with existing name.");
    vertexNames.put(v.name, v);
  }

  /**
   * Gets a collection of all the vertices in the graph
   * 
   * @return (Collection<Vertex>) collection of all the vertices in the graph
   */
  public Collection<Vertex> getVertices() {
    return vertexNames.values();
  }

  /**
   * Gets the vertex object with the given name
   * 
   * @param name
   *          (String) name of the vertex object requested
   * @return (Vertex) vertex object associated with the name
   */
  public Vertex getVertex(String name) {
    return vertexNames.get(name);
  }

  /**
   * Adds a directed edge from vertex u to vertex v
   * 
   * @param nameU
   *          (String) name of vertex u
   * @param nameV
   *          (String) name of vertex v
   * @param cost
   *          (double) cost of the edge between vertex u and v
   */
  public void addEdge(int nameU, int nameV, Double cost) {
    if (!vertexNames.containsKey(nameU))
      throw new IllegalArgumentException(nameU + " does not exist. Cannot create edge.");
    if (!vertexNames.containsKey(nameV))
      throw new IllegalArgumentException(nameV + " does not exist. Cannot create edge.");
    Vertex sourceVertex = vertexNames.get(nameU);
    Vertex targetVertex = vertexNames.get(nameV);
    Edge newEdge = new Edge(sourceVertex, targetVertex, cost);
    sourceVertex.addEdge(newEdge);
  }

  /**
   * Adds an undirected edge between vertex u and vertex v by adding a directed
   * edge from u to v, then a directed edge from v to u
   * 
   * @param name
   *          (String) name of vertex u
   * @param name2
   *          (String) name of vertex v
   * @param cost
   *          (double) cost of the edge between vertex u and v
   */
  public void addUndirectedEdge(int name, int name2, double cost) {
    addEdge(name, name2, cost);
    addEdge(name2, name, cost);
  }


  /**
   * Computes the euclidean distance between two points as described by their
   * coordinates
   * 
   * @param ux
   *          (double) x coordinate of point u
   * @param uy
   *          (double) y coordinate of point u
   * @param vx
   *          (double) x coordinate of point v
   * @param vy
   *          (double) y coordinate of point v
   * @return (double) distance between the two points
   */
  public double computeEuclideanDistance(double ux, double uy, double vx, double vy) {
    return Math.sqrt(Math.pow(ux - vx, 2) + Math.pow(uy - vy, 2));
  }

  /**
   * Computes euclidean distance between two vertices as described by their
   * coordinates
   * 
   * @param u
   *          (Vertex) vertex u
   * @param v
   *          (Vertex) vertex v
   * @return (double) distance between two vertices
   */
  public double computeEuclideanDistance(Vertex u, Vertex v) {
    return computeEuclideanDistance(u.x, u.y, v.x, v.y);
  }

  /**
   * Calculates the euclidean distance for all edges in the map using the
   * computeEuclideanCost method.
   */
  public void computeAllEuclideanDistances() {
    for (Vertex u : getVertices())
      for (Edge uv : u.adjacentEdges) {
        Vertex v = uv.target;
        uv.distance = computeEuclideanDistance(u.x, u.y, v.x, v.y);
      }
  }



  // STUDENT CODE STARTS HERE

  
  public void generateRandomVertices(int n) {
	  vertexNames = new HashMap<>(); // reset the vertex hashmap
	  Random generator = new Random();
      for (int i = 0; i<n; i++) {
    	  int x = generator.nextInt(101);
    	  int y = generator.nextInt(101);
    	  vertexNames.put(i, new Vertex(i, x, y));
      }
      
      Collection<Vertex> values = vertexNames.values();
      
	  Iterator<Vertex> itr = values.iterator();
	  Iterator<Vertex> itr2 = values.iterator();
	  
	  while (itr.hasNext()) {
		  Vertex first = itr.next();	
		  while (itr2.hasNext()) {
			  
			  
		  Vertex second = itr2.next();
		  addUndirectedEdge(first.name, second.name, computeEuclideanDistance(first.x, first.y, second.x, second.y));
	  }
		  
		  itr2 = values.iterator();
  }
      
      
      
	  computeAllEuclideanDistances(); // compute distances
  }

  
  public List<Edge> nearestNeighborTsp() {
    boolean foundMin = false;
    boolean first;
	  vertices = vertexNames.values();
    for (Vertex v : vertices) {
    	System.out.println("vertex" + v);
    }
    
    HashMap<Double, List<Edge>> paths = new HashMap<>();
	  
	  for (Vertex v : vertices) {
		  
		  List<Edge> edges = new ArrayList<>();
		  double distance = 0;
		  
		  Vertex originalVertex = v;
		  
		  first = true;
		  
			  while ((v!=originalVertex || first) && v!=null) {
				  
			  Edge minEdge = new Edge(null, null, 0);
			  double minDistance = Double.POSITIVE_INFINITY;
			  
			  for (Edge e : v.adjacentEdges) {
				  
				  if (e.distance<minDistance && !e.target.known && e.target!=v) {
					  
					  minDistance = e.distance;
					  minEdge = e;
					  foundMin = true;
				  }
					  }
			  
			  if (foundMin==false) {
				  
				  break;
			  }
			  else {
				  distance+=minEdge.distance;
			  
			  edges.add(minEdge);
			  
		  paths.put(distance, edges);
		  
		  v.known = true;
		  v=minEdge.target;
		  v.known = true;
		  
		  first = false;
				foundMin = false;  
			  }
			  
			  }
			  }
	  
	  
  
	  
	  Collection<Double> distances = paths.keySet();
	  
	  double minDistance = Collections.min(distances);
	  List<Edge> minPath = paths.get(minDistance);
	  Vertex lastVert = minPath.get(minPath.size()-1).target;
	  Vertex firstVert = minPath.get(0).source;
	  minPath.add(new Edge(lastVert, firstVert, computeEuclideanDistance(lastVert.x, lastVert.y, firstVert.x, firstVert.y)));
	  
	  return minPath;
	  
			
		  

    
  }

  
  
  public List<Edge> bruteForceTsp() {
	  return null;
  
  
  }
	
	  
	  
		  

		  // STUDENT CODE ENDS HERE



  /**
   * Prints out the adjacency list of the graph for debugging
   */
  public void printAdjacencyList() {
    for (int u : vertexNames.keySet()) {
      StringBuilder sb = new StringBuilder();
      sb.append(u);
      sb.append(" -> [ ");
      for (Edge e : vertexNames.get(u).adjacentEdges) {
        sb.append(e.target.name);
        sb.append("(");
        sb.append(e.distance);
        sb.append(") ");
      }
      sb.append("]");
      System.out.println(sb.toString());
    }
  }
}

