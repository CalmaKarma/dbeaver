/*
 * DBeaver - Universal Database Manager
 * Copyright (C) 2010-2022 DBeaver Corp and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.jkiss.dbeaver.erd.ui.router;


import org.eclipse.draw2dl.geometry.Point;
import org.eclipse.draw2dl.geometry.PointList;
import org.eclipse.draw2dl.geometry.PrecisionPoint;
import org.eclipse.draw2dl.geometry.Rectangle;

import org.jkiss.code.NotNull;
import org.jkiss.code.Nullable;
import org.jkiss.utils.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Mikami-Tabuchi’s Algorithm
 * 1. Expand horizontal and vertical line from source to target
 * 2. In every iteration, expand from the last expanded line by STEP_SIZE
 * 3. Continue until a line from source intersects another line from target
 * 4. Backtrace from interception
 */
//possible optimizations
//By the rules of math parallel lines couldn't collide, so we need to check only perpendicular lines of opposite source/target origin
//multi-dimensional arrays for trial lines?
public class MikamiTabuchiRouter {

    private int spacing = 15;
    private List<Rectangle> obstacles = new ArrayList<>();
    private PrecisionPoint start, finish;
    private final List<OrthogonalPath> userPaths = new ArrayList<>();

    //Increase for performance, increasing this parameter lowers accuracy.
    private static final int STEP_SIZE = 4;


    private static final int SOURCE_VERTICAL_LINES = 0;
    private static final int SOURCE_HORIZONTAL_LINES = 1;
    private static final int TARGET_VERTICAL_LINES = 2;
    private static final int TARGET_HORIZONTAL_LINES = 3;

    private Map<Integer, Map<Integer, List<TrialLine>>> linesMap;

    //In worst case scenarios line search may become laggy,
    //if after this amount iterations nothing was found -> stop
    private static final int MAX_ITER = 5;

    Rectangle clientArea;

    public void setObstacles(List<Rectangle> obstacles) {
        this.obstacles = obstacles;
    }

    public void setClientArea(Rectangle clientArea) {
        this.clientArea = clientArea;
    }



    private Pair<TrialLine, TrialLine> result;

    private void createLinesFromTrial(TrialLine pos, int iter) {
        //possible optimisation
        //We don't want to create line if line of the same orientation already crosses this point.
        float from = pos.vertical ? pos.from.y : pos.from.x;
        float start = pos.start;
        float end = pos.finish;
        for (float i = (pos.hasForbiddenRange() ? pos.creationForbiddenStart - 1 : from); i >= start; i -= STEP_SIZE) {
            if (createTrial(pos, iter, i)) {
                return;
            }
        }
        for (float i = (pos.hasForbiddenRange() ? pos.creationForbiddenFinish + 1 : from); i < end; i += STEP_SIZE) {
            if (createTrial(pos, iter, i)) {
                return;
            }
        }
    }

    private boolean createTrial(TrialLine pos, int iter, float i) {
        TrialLine trialLine = createTrialLine(i, !pos.vertical, pos);
        getLinesMap(trialLine, iter).add(trialLine);
        final TrialLine interception = trialLine.findInterception();
        // We found needed line, finish execution
        if (interception != null) {
            result = new Pair<>(trialLine, interception);
            return true;
        }
        return false;
    }

    @NotNull
    private List<TrialLine> getLinesMap(TrialLine line, int iteration) {
        if (line.vertical) {
            return line.fromSource ? linesMap.get(iteration).get(SOURCE_VERTICAL_LINES) : linesMap.get(iteration).get(TARGET_VERTICAL_LINES);
        } else {
            return line.fromSource ? linesMap.get(iteration).get(SOURCE_HORIZONTAL_LINES) : linesMap.get(iteration).get(TARGET_HORIZONTAL_LINES);
        }
    }

    @NotNull
    private List<TrialLine> getOpposingLinesMap(TrialLine line, int iteration) {
        if (line.vertical) {
            return line.fromSource ? linesMap.get(iteration).get(TARGET_HORIZONTAL_LINES) : linesMap.get(iteration).get(SOURCE_HORIZONTAL_LINES);
        } else {
            return line.fromSource ? linesMap.get(iteration).get(TARGET_VERTICAL_LINES) : linesMap.get(iteration).get(SOURCE_VERTICAL_LINES);
        }
    }

    private PrecisionPoint getInterceptionPoint(TrialLine source, TrialLine target) {
        if (source.vertical) {
            return new PrecisionPoint(source.from.x, target.from.y);
        } else {
            return new PrecisionPoint(target.from.x, source.from.y);
        }
    }

    private TrialLine createTrialLine(float pos, boolean vertical, @NotNull TrialLine parentLine) {
        final TrialLine trialLine;
        if (vertical) {
            trialLine = new TrialLine(new PrecisionPoint(pos, parentLine.from.y), parentLine);
        } else {
            trialLine = new TrialLine(new PrecisionPoint(parentLine.from.x, pos), parentLine);
        }
        return trialLine;
    }

    public void setSpacing(int spacing) {
        this.spacing = spacing;
    }

    public boolean updateObstacle(Rectangle rectangle, Rectangle newBounds) {
        boolean result = obstacles.remove(rectangle);
        result |= obstacles.add(newBounds);
        return result;
    }

    public void addObstacle(Rectangle bounds) {
        obstacles.add(bounds);
    }

    public boolean removeObstacle(Rectangle bounds) {
        return obstacles.remove(bounds);
    }

    private PointList traceback() {
        PointList points = new PointList();
        TrialLine line = result.getFirst();
        PrecisionPoint point = null;
        while (line != null) {
            if (point == null || !point.equals(line.from)) {
                points.addPoint(line.from);
                point = line.from;
            }
            line = line.getParent();
        }
        points.reverse();
        point = getInterceptionPoint(result.getFirst(), result.getSecond());
        points.addPoint(point);
        line = result.getSecond();
        while (line != null) {
            if (!line.from.equals(point)) {
                points.addPoint(line.from);
                point = line.from;
            }
            line = line.getParent();
        } ;
        return points;
    }


    public List<OrthogonalPath> solve() {
        List<OrthogonalPath> updated = new ArrayList<>();
        for (OrthogonalPath userPath : userPaths.stream().filter(OrthogonalPath::isDirty).collect(Collectors.toList())) {
            final PointList pointList = solveConnection(userPath.getStart(), userPath.getEnd());
            userPath.setBendPoints(pointList);
            updated.add(userPath);
        }
        return updated;
    }

    @org.jkiss.code.Nullable
    private PointList solveConnection(Point start, Point finish) {
        if (start.equals(finish)) {
            return null;
        }
        linesMap = new HashMap<>();
        this.start = new PrecisionPoint(start);
        result = null;
        this.finish = new PrecisionPoint(finish);
        int iter = 0;
        initStartingTrialLines();
        if (result != null) {
            return traceback();
        }
        while (iter != MAX_ITER && result == null) {
            linesMap.put(iter + 1, new HashMap<>());
            initNewLayer(iter + 1);
            for (int i = 0; i < 4; i++) {
                for (TrialLine trialLine : linesMap.get(iter).get(i)) {
                    createLinesFromTrial(trialLine, iter + 1);
                    if (result != null) {
                        return traceback();
                    }
                }
            }
            iter++;
        }
        return null;
    }



    private void initStartingTrialLines() {
        //Deviation from an original algorithm, we want only lines what connect with point horizontally
        final TrialLine horizontalStartTrial = new TrialLine(start, true, false);
        final TrialLine horizontalFinishTrial = new TrialLine(finish, false, false);

        linesMap.put(0, new HashMap<>());
        initNewLayer(0);
        linesMap.get(0).get(SOURCE_HORIZONTAL_LINES).add(horizontalStartTrial);
        linesMap.get(0).get(TARGET_HORIZONTAL_LINES).add(horizontalFinishTrial);
    }

    /**
     * inits list for each type of
     * @param iter number of algorithm iteration
     */
    private void initNewLayer(int iter) {
        for (int i = 0; i < 4; i++) {
            linesMap.get(iter).put(i, new ArrayList<>());
        }
    }

    public void removePath(OrthogonalPath path) {
        this.userPaths.remove(path);
    }

    public void addPath(OrthogonalPath path) {
        this.userPaths.add(path);
    }

    private class TrialLine {

        float start = Integer.MIN_VALUE;
        float finish = Integer.MIN_VALUE;
        boolean fromSource;

        int creationForbiddenStart = Integer.MIN_VALUE;
        int creationForbiddenFinish = Integer.MIN_VALUE;
        final PrecisionPoint from;


        boolean vertical;

        //Starting line is always inside figure, we don't want to create trial line inside it
        private void calculateForbiddenRange() {
            for (Rectangle it : obstacles) {
                if (isInsideFigure(it, false)) {
                    if (vertical) {
                        creationForbiddenStart = it.y - spacing;
                        creationForbiddenFinish = it.y + it.height + spacing;
                    } else {
                        creationForbiddenStart = it.x - spacing;
                        creationForbiddenFinish = it.x + it.width + spacing;
                    }
                }
            }
        }

        public boolean hasForbiddenRange() {
            return creationForbiddenFinish != Integer.MIN_VALUE;
        }


        public int getCreationForbiddenStart() {
            return creationForbiddenStart;
        }

        public int getCreationForbiddenFinish() {
            return creationForbiddenFinish;
        }

        @Nullable
        TrialLine parent;

        TrialLine(PrecisionPoint start, @NotNull TrialLine parent) {
            this.from = start;
            this.parent = parent;
            this.fromSource = parent.fromSource;
            this.vertical = !parent.vertical;
            cutByObstacles(false);
        }

        TrialLine(PrecisionPoint start, boolean fromSource, boolean vertical) {
            this.from = start;
            this.vertical = vertical;
            this.fromSource = fromSource;
            this.cutByObstacles(true);
            this.calculateForbiddenRange();
        }

        private boolean isInsideFigure(Rectangle it, boolean ignoreOffset) {
            int offset = spacing;
            if (ignoreOffset) {
                offset = 0;
            }
            return (it.x - offset <= from.x && it.x + it.width + offset > from.x
                && it.y - offset <= from.y && it.y + it.height + offset > from.y);
        }

        private void cutByObstacles(boolean startingLine) {
            //Check if object is on axis with line, if it is, reduce line size
            for (Rectangle it : obstacles) {
                if (isInsideFigure(it, true) && startingLine) {
                    continue;
                }
                if (it.x - spacing <= from.x && it.x + it.width + spacing > from.x
                    || it.y - spacing <= from.y && it.y + it.height + spacing > from.y) {
                    //object is below need to cut start
                    cut(it);
                }
            }
            if (finish == Integer.MIN_VALUE) {
                if (vertical) {
                    finish = clientArea.height - spacing;
                } else {
                    finish = clientArea.width - spacing;
                }
            }
            if (start == Integer.MIN_VALUE) {
                start = spacing;
            }
        }

        private void cut(Rectangle bound) {
            int fromPosition = vertical ? from.y : from.x;
            int obstaclePosition = vertical ? bound.y : bound.x;
            int obstacleSize = vertical ? bound.height : bound.width;
            if (fromPosition > obstaclePosition + obstacleSize + spacing) {
                if (start == Integer.MIN_VALUE || start < obstaclePosition + spacing) {
                    start = obstaclePosition + obstacleSize + spacing;
                }
            }
            //object is above, need to cut finish
            if (fromPosition  <= obstaclePosition - spacing) {
                if (finish == Integer.MIN_VALUE || finish > obstaclePosition - spacing) {
                    finish = obstaclePosition - spacing;
                }
            }
        }

        @Nullable
        public TrialLine findInterception() {
            for (int i = linesMap.values().size() - 1; i >= 0; i--) {
                for (TrialLine trialLine : getOpposingLinesMap(this, i)) {
                    if (intercept(trialLine)) {
                        return trialLine;
                    }
                }
            }
            return null;
        }

        private boolean intercept(TrialLine line) {
            int firstLinePos = vertical ? from.y : from.x;
            int secondLinePos = vertical ? line.from.x : line.from.y;
            return firstLinePos >= line.start && firstLinePos < line.finish && secondLinePos >= start && secondLinePos < finish;
        }

        @Nullable
        public TrialLine getParent() {
            return parent;
        }
    }
}