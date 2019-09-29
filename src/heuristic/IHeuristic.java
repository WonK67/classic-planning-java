package heuristic;

import fr.uga.pddl4j.heuristics.relaxation.Heuristic;

public interface IHeuristic extends Heuristic {

	enum Type {
        FAST_FORWARD,
        SUM,
        MAX,
        ONE_FOR_ALL

    }
	
}
