package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

public class SelectableFollowerAction {
    private final Selector<FollowerAction> selector;
    private final LinkedList<FollowerAction> followerActionList = new LinkedList<>();

    public SelectableFollowerAction(String name, Consumer<SelectScope<FollowerAction>> actions) {
        String[] message = {
                "Move D-pad cursor to choose robot's next action.",
                "Press right bumper to add the selected item.",
                "Press left bumper to delete the last selected."
        };
        selector = Selector.create(name, actions, message);
        selector.onSelect(followerActionList::addLast);
    }

    public void decrementSelected() {
        selector.decrementSelected();
    }

    public void incrementSelected() {
        selector.incrementSelected();
    }

    public void addSelected() {
        selector.select();
    }

    public void removeLast() {
        if (!followerActionList.isEmpty())
            followerActionList.removeLast();
    }

    public List<String> getSelectableLines() {
        return selector.getLines();
    }

    public String getActionLines() {
        StringBuilder lines = new StringBuilder();
        for (int i = 0; i < followerActionList.size(); i++) {
            lines.append(i != 0 ? " -> " : "").append(followerActionList.get(i).toString());
        }
        return lines.toString();
    }

    public FollowerAction getNextAction() {
        return followerActionList.poll();
    }
}

