package heuristic;

import fr.uga.pddl4j.heuristics.relaxation.Heuristic;

public interface IHeuristic extends Heuristic {

	enum Type {
        /**
         * The type for the <code>Max</code> heuristic.
         */
        MAX,
        /**
         * The type for the <code>FastForward</code> heuristic.
         */
        FAST_FORWARD,
        /**
         * The type for the <code>Sum</code> heuristic.
         */
        SUM,
        ONE_FOR_ALL

    }
	
}
