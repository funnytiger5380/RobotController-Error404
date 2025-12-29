package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

/**
 * The SelectableFollowerAction class builds a user chosen Follower's action sequence from a
 * predefined set (FollowerAction enum) which the Follower follows at the runtime.
 *
 */
public class SelectableFollowerAction {
    private final Selector<FollowerAction> selector;
    private final LinkedList<FollowerAction> followerActionList = new LinkedList<>();

    /** The constructor for the SelectableFollowerAction class. User can add FollowerAction
     * selection from the FollowAction enum class to form a sequence of actions which the Follower
     * follows at the runtime.
     * The proper usage is:
     * SelectableFollowerAction followerSelectedAction = new SelectableFollowerAction(
     "<< Follower Action Sequence Selection >>", a -> {
     a.add("String name", FollowerAction.<enum>);
     a.add()...
     */
    public SelectableFollowerAction(String name, Consumer<SelectScope<FollowerAction>> actions) {
        String[] message = {
                "Move D-pad cursor to choose robot's next action.",
                "Press right bumper to add the selected item.",
                "Press left bumper to delete the last selected."
        };
        selector = Selector.create(name, actions, message);
        selector.onSelect(followerActionList::addLast);
    }

    /** Display the selection cursor decrement (move up) at Telemetry.
     */
    public void decrementSelected() {
        selector.decrementSelected();
    }

    /** Display the selection cursor increment (move down) at Telemetry.
     */
    public void incrementSelected() {
        selector.incrementSelected();
    }

    /** Chosen selection is added to the followerAction sequence for the Follower.
     */
    public void addSelected() {
        selector.select();
    }

    /** Remove the last action selection from the followerAction sequence.
     */
    public void removeLast() {
        if (!followerActionList.isEmpty())
            followerActionList.removeLast();
    }

    /** Telemetry helper function to display user defined followerAction selection from the
     * SelectableFollowerAction class constructor.
     */
    public List<String> getSelectableLines() {
        return selector.getLines();
    }

    /** Telemetry helper function to display user chosen followerAction sequence.
     */
    public String getActionLines() {
        StringBuilder lines = new StringBuilder();
        for (int i = 0; i < followerActionList.size(); i++) {
            lines.append(i != 0 ? " -> " : "").append(followerActionList.get(i).toString());
        }
        return lines.toString();
    }

    /** Returns the next action from the followerAction sequence. The return action is removed from
     * the followerAction sequence.
     */
    public FollowerAction getNextAction() {
        return followerActionList.poll();
    }
}