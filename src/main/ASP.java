package main;
import java.io.File;
import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Properties;
import java.util.Set;
import java.util.Comparator;

import fr.uga.pddl4j.encoding.CodedProblem;
import fr.uga.pddl4j.parser.ErrorManager;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.ProblemFactory;
import fr.uga.pddl4j.planners.Statistics;
import fr.uga.pddl4j.planners.statespace.AbstractStateSpacePlanner;
import fr.uga.pddl4j.planners.statespace.StateSpacePlanner;
import fr.uga.pddl4j.planners.statespace.search.strategy.StateSpaceStrategy;
import fr.uga.pddl4j.util.BitOp;
import fr.uga.pddl4j.util.BitState;
import fr.uga.pddl4j.util.CondBitExp;
import fr.uga.pddl4j.util.MemoryAgent;
import fr.uga.pddl4j.util.Plan;
import fr.uga.pddl4j.util.SequentialPlan;
import heuristic.HeuristicCreator;
import heuristic.IHeuristic;
import fr.uga.pddl4j.heuristics.relaxation.Heuristic;
import fr.uga.pddl4j.planners.statespace.search.strategy.AStar;
import fr.uga.pddl4j.planners.statespace.search.strategy.Node;
import fr.uga.pddl4j.planners.statespace.search.strategy.StateSpaceStrategy;

public final class ASP extends AbstractStateSpacePlanner {
	
  /*
   * The arguments of the planner.
   */
  private Properties arguments;
	
  /**
   * Creates a new ASP planner with the default parameters.
   * 
   * @param arguments the arguments of the planner.
   */
  public ASP(final Properties arguments) {
    super();
    this.arguments = arguments;
  }

  /**
   * Solves the planning problem and returns the first solution search found.
   *
   * @param problem the problem to be solved.
   * @return a solution search or null if it does not exist.
   */
  /*@Override
  public Plan search(final CodedProblem problem) {
    int timeout = (int) this.arguments.get(Planner.TIMEOUT);
    double weight = (double) arguments.get(StateSpacePlanner.WEIGHT);         
    StateSpaceStrategy astar = new AStar(timeout, Heuristic.Type.FAST_FORWARD, weight);
    Node goalNode = astar.searchSolutionNode(problem);
    Planner.getLogger().trace(problem.toString(goalNode));
    return astar.extractPlan(goalNode, problem);
  }
  
  public Plan search(final CodedProblem problem, IHeuristic.Type heuristic) {
	    int timeout = (int) this.arguments.get(Planner.TIMEOUT);
	    double weight = (double) arguments.get(StateSpacePlanner.WEIGHT);         
	    StateSpaceStrategy astar = new AStar(timeout, heuristic, weight);
	    Node goalNode = astar.searchSolutionNode(problem);
	    Planner.getLogger().trace(problem.toString(goalNode));
	    return astar.extractPlan(goalNode, problem);
  }*/
  
  @Override
  public Plan search(final CodedProblem problem) {
	  
    // First we create an instance of the heuristic to use to guide the search
    final IHeuristic heuristic = HeuristicCreator.createHeuristic(IHeuristic.Type.FAST_FORWARD, problem);
    
    // We get the initial state from the planning problem
    final BitState init = new BitState(problem.getInit());
          
    // We initialize the closed list of nodes (store the nodes explored)
    final Set<Node> close = new HashSet<>();
          
    // We initialize the opened list to store the pending node according to function f
    final double weight = (double) arguments.get(StateSpacePlanner.WEIGHT);
    final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
        public int compare(Node n1, Node n2) {
            double f1 =  weight * n1.getHeuristic() + n1.getCost();
            double f2 = weight * n2.getHeuristic() + n2.getCost();
            return Double.compare(f1, f2);
          }
        });
          
    // We create the root node of the tree search
    final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));
     
    // We adds the root to the list of pending nodes
    open.add(root);
    Plan plan = null;

     final int timeout = ((int) this.arguments.get(Planner.TIMEOUT)) * 1000;
    long time = 0;
          
    // We start the search
    while (!open.isEmpty() && plan == null && time < timeout) {
       
      // We pop the first node in the pending list open
      final Node current = open.poll();
      close.add(current);
              
      // If the goal is satisfy in the current node then extract the search and return it
      if (current.satisfy(problem.getGoal())) {
        return this.extractPlan(current, problem);
      } 
              
      // Else we try to apply the operators of the problem to the current node
      else {    
        for (int i = 0; i < problem.getOperators().size(); i++) {
          // We get the its operator of the problem
          BitOp a = problem.getOperators().get(i);
          // If the operator is applicable in the current node
          if (a.isApplicable(current)) {
            Node next = new Node(current);
            // We apply the effect of the operator
            final List<CondBitExp> effects = a.getCondEffects();
            for (CondBitExp ce : effects) {
              if (current.satisfy(ce.getCondition())) {
                next.apply(ce.getEffects());
              }
            }
            // We set the new child node information
            final double g = current.getCost() + 1;
            if (!close.contains(next)) {
              next.setCost(g);
              next.setParent(current);
              next.setOperator(i);
              next.setHeuristic(heuristic.estimate(next, problem.getGoal()));
              open.add(next);
            }
          }
        }
      }
    }

    // We compute the memory by the search
    this.getStatistics().setMemoryUsedToSearch(MemoryAgent.sizeOf(open) + MemoryAgent.sizeOf(close));
     
    // Finally, we return the search computed or null if no search was found
    return plan;   
  }
  
  private Plan extractPlan(final Node node, final CodedProblem problem) {
	  Node n = node;
	  final Plan plan = new SequentialPlan();
	  while (n.getOperator() != -1) {
	    final BitOp op = problem.getOperators().get(n.getOperator());
	    plan.add(0, op);
	    n = n.getParent();
	  }
	  return plan;
	}

 /**
  * The main method of the <code>ASP</code> example. The command line syntax is as 
  * follow:
  * <p>
  * <pre>
  * usage of ASP:
  *
  * OPTIONS   DESCRIPTIONS
  *
  * -o <i>str</i>   operator file name
  * -f <i>str</i>   fact file name
  * -w <i>num</i>   the weight used in the a star search (preset: 1)
  * -t <i>num</i>   specifies the maximum CPU-time in seconds (preset: 300)
  * -h              print this message
  *
  * </pre>
  * </p>
  *
  * @param args the arguments of the command line.
  */
  public static void main(String[] args) {
	  
	  IHeuristic.Type[] heuristics = {IHeuristic.Type.FAST_FORWARD, IHeuristic.Type.ONE_FOR_ALL};
	  
	  for(IHeuristic.Type heuristic: heuristics) {
		  
	  System.out.println("Executando A* para heurística: " + heuristic.name());
	  
	  final Properties arguments = ASP.parseCommandLine(args);
	  if (arguments == null) {
	    ASP.printUsage();
	    System.exit(0);
	  }
	  
	  final ASP planner = new ASP(arguments);
	  final ProblemFactory factory = ProblemFactory.getInstance();
	  
	  File domain = (File) arguments.get(Planner.DOMAIN);
	  File problem = (File) arguments.get(Planner.PROBLEM);
	  ErrorManager errorManager = null;
	  try {
	    errorManager = factory.parse(domain, problem);
	  } catch (IOException e) {
	    Planner.getLogger().trace("\nunexpected error when parsing the PDDL planning problem description.");
	    System.exit(0);
	  }
	  
	  if (!errorManager.isEmpty()) {
		  errorManager.printAll();
		  System.exit(0);
		} else {
		  Planner.getLogger().trace("\nparsing domain file done successfully");
		  Planner.getLogger().trace("\nparsing problem file done successfully\n");
		}
	  
	  final CodedProblem pb = factory.encode();
	  Planner.getLogger().trace("\nencoding problem done successfully (" 
  		    + pb.getOperators().size() + " ops, "
  		    + pb.getRelevantFacts().size() + " facts)\n");
	  
	  if (!pb.isSolvable()) {
		  Planner.getLogger().trace(String.format("goal can be simplified to FALSE." 
		                                            +  "no search will solve it%n%n"));
		  System.exit(0);
		}
	  
	  long begin = System.currentTimeMillis();
	  final Plan plan = planner.search(pb);
	  planner.getStatistics().setTimeToSearch(System.currentTimeMillis() - begin);
	  planner.getStatistics().setMemoryUsedForProblemRepresentation(MemoryAgent.getDeepSizeOf(pb));
	  
	  if (plan != null) {
	    // Print plan information
	    Planner.getLogger().trace(String.format("%nfound plan as follows:%n%n" + pb.toString(plan)));
	    Planner.getLogger().trace(String.format("%nplan total cost: %4.2f%n%n", plan.cost()));
	  } else {
	    Planner.getLogger().trace(String.format(String.format("%nno plan found%n%n")));
	  }
	  
	// Get the runtime information from the planner
	  Statistics info = planner.getStatistics();
	      	
	  // Print time information
	  long time = info.getTimeToParse() +  info.getTimeToEncode() + info.getTimeToSearch();
	  Planner.getLogger().trace(String.format("%ntime spent:   %8.4f seconds parsing %n", info.getTimeToParse()/1000.0));
	  Planner.getLogger().trace(String.format("              %8.4f seconds encoding %n", info.getTimeToEncode()/1000.0));
	  Planner.getLogger().trace(String.format("              %8.4f seconds searching%n", info.getTimeToSearch()/1000.0));
	  Planner.getLogger().trace(String.format("              %8.4f seconds total time%n", time/1000.0));
	      	
	  // Print memory usage information
	  long memory = info.getMemoryUsedForProblemRepresentation() + info.getMemoryUsedToSearch();
	  Planner.getLogger().trace(String.format("%nmemory used:  %8.2f MBytes for problem representation%n", info.getMemoryUsedForProblemRepresentation()/(1024.0*1024.0)));
	  Planner.getLogger().trace(String.format("              %8.2f MBytes for searching%n", info.getMemoryUsedToSearch()/(1024.0*1024.0)));
	  Planner.getLogger().trace(String.format("              %8.2f MBytes total%n%n%n", memory/(1024.0*1024.0)));
	  }
  }
  
  
  /**
   * Print the usage of the ASP planner.
   */
  private static void printUsage() {
    final StringBuilder strb = new StringBuilder();
    strb.append("\nusage of PDDL4J:\n")
      .append("OPTIONS   DESCRIPTIONS\n")
      .append("-o <str>    operator file name\n")
      .append("-f <str>    fact file name\n")
      .append("-w <num>    the weight used in the a star seach (preset: 1.0)\n")
      .append("-t <num>    specifies the maximum CPU-time in seconds (preset: 300)\n")
      .append("-h          print this message\n\n");
    Planner.getLogger().trace(strb.toString());
  }
  
  
  /**
   * Parse the command line and return the planner's arguments.
   * 
   * @param args the command line.
   * @return the planner arguments or null if an invalid argument is encountered.
   */
  
  private static Properties parseCommandLine(String[] args) {
    
    // Get the default arguments from the super class 
    final Properties arguments = StateSpacePlanner.getDefaultArguments();
    
    // Parse the command line and update the default argument value 
    for (int i = 0; i < args.length; i += 2) {
      if ("-o".equalsIgnoreCase(args[i]) && ((i + 1) < args.length)) {
        if (!new File(args[i + 1]).exists()) return null;
        arguments.put(Planner.DOMAIN, new File(args[i + 1]));
      } else if ("-f".equalsIgnoreCase(args[i]) && ((i + 1) < args.length)) {
        if (!new File(args[i + 1]).exists()) return null;
        arguments.put(Planner.PROBLEM, new File(args[i + 1]));
      } else if ("-t".equalsIgnoreCase(args[i]) && ((i + 1) < args.length)) {
        final int timeout = Integer.parseInt(args[i + 1]) * 1000;
        if (timeout < 0) return null;
        arguments.put(Planner.TIMEOUT, timeout);
      } else if ("-w".equalsIgnoreCase(args[i]) && ((i + 1) < args.length)) {
        final double weight = Double.parseDouble(args[i + 1]);
        if (weight < 0) return null;
        arguments.put(StateSpacePlanner.WEIGHT, weight);
      } else {
        return null;
      }
    }
    // Return null if the domain or the problem was not specified 
    return (arguments.get(Planner.DOMAIN) == null 
        || arguments.get(Planner.PROBLEM) == null) ? null : arguments;
  }

}