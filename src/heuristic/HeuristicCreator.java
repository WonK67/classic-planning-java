package heuristic;

import fr.uga.pddl4j.encoding.CodedProblem;
import fr.uga.pddl4j.heuristics.relaxation.FastForward;
import fr.uga.pddl4j.heuristics.relaxation.Heuristic;
import fr.uga.pddl4j.heuristics.relaxation.HeuristicToolKit;
import fr.uga.pddl4j.heuristics.relaxation.Max;
import fr.uga.pddl4j.heuristics.relaxation.Sum;

public class HeuristicCreator {

    /**
     * The serial version id of the class.
     */
    private static final long serialVersionUID = 1L;

    /**
     * Private constructor just for prevent user to instantiate this class.
     */
    private HeuristicCreator() {
    }

    /**
     * Create an heuristic of a specified type.
     *
     * @param type    the type of the heuristic to create.
     * @param problem the problem for which the heuristic is created.
     * @return the heuristic created.
     * @throws NullPointerException if <code>type == null || problem == null</code>.
     */
    public static IHeuristic createHeuristic(final IHeuristic.Type type, final CodedProblem problem) {
        IHeuristic heuristic;
        if (type.equals(IHeuristic.Type.FAST_FORWARD)) {
            heuristic = new FastForwardHeuristic(problem);
        } else if (type.equals(IHeuristic.Type.ONE_FOR_ALL)) {
            heuristic = new OneForAllHeuristic();
        } else if (type.equals(IHeuristic.Type.SUM)) {
            heuristic = new SumHeuristic(problem);
        } else if (type.equals(IHeuristic.Type.MAX)) {
            heuristic = new MaxHeuristic(problem);
        } else {
        	throw new UnsupportedOperationException();
        }
        return heuristic;
    }
}
