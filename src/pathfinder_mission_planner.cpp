#include "pathfinder_mission_planner.h"
#include "navigator_2d.h"

PathfinderMissionPlanner::PathfinderMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator) : BaseMissionPlanner(startPosition, costEstimator, navigator) {}

//PathfinderMissionPlanner::PathfinderMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator) : currentPosition_(startPosition), currentWaypoint_(navigator->GetClosestWaypointName(startPosition)), costEstimator_(costEstimator), navigator_(navigator), currentPower_(startPosition.power) {}

// random search function which is called to find a valid path if GetNextWaypoint fails
std::vector<std::string> PathfinderMissionPlanner::PanicSearch(const std::string& currWaypoint, const std::string& goal) 
{
    std::string nextPoint;                                                              
    std::string curr = currWaypoint;
    std::string op;
    std::vector<std::string> OldMoves;  // vector for past moves that led to dead ends
    std::vector<std::string> wp_names = navigator_->GetAllWaypointNames();
    std::map<std::string, Point2D> waypoints2;

    // copy of waypoints as waypoints2 map
    for (const auto& point : wp_names)
    {
        waypoints2[point] = navigator_->GetWaypointPosition(point);
    }

    int pwr = currentPower_;
    int count = 0;  // used to check if finding first move in a valid path
    bool fwd = true;
    int count2 = 1; // counts total number of iterations to find valid path

    std::vector<std::string> PossibleMoves;
    std::vector<std::string> Path;

    // checks if the goal is a possible move and skips while loop by changing fwd to false and returns goal as next move
    if (navigator_->GetDistanceToWaypoint(waypoints2[currWaypoint], goal) <= pwr)
    {
        nextPoint = goal;
        Path.push_back(" ");
        Path.push_back(nextPoint);
        fwd = false;
    }

    // fwd true unless goal is reachable in one move
    if (fwd == true)
    {
        // loops until valid path is found
        while (curr != goal)
        {
            // clears possible moves vector after "changing" position
            PossibleMoves.clear();

            // waypoints added to possible moves if they are within reach, are not past moves in missionPlan (to prevent going back to start), and are not the same as "current position"
            for (const auto& moves : waypoints2)
            {
                if ((navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) <= pwr) && (std::find(missionPlan_.begin(), missionPlan_.end(), moves.first) == missionPlan_.end()) && (navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) != 0))
                {
                    PossibleMoves.push_back(moves.first);
                }
            }

            // if finding first move in a valid path and possible moves is empty -> mission failure
            if (((PossibleMoves.size() == 0) && (count < 1)) || (count2 > 50))
            {
                pwr = 0;
                fwd = false;
                std::cout << "[PanicSearch] Mission Failure" <<std::endl;
                Path.clear();
                Path.push_back("");
                Path.push_back("");
                return Path;
                
            }
            // if not finding first move in a valid path and possible moves is empty -> reset power to 0 to trigger reset to beginning and look for another path
            else if ((PossibleMoves.size() == 0) && (count > 0))
            {
                pwr = 0;
            }
            // picks random move from possible moves and changes curr to that waypoint, adds move old curr to path, and adds/subtracts power
            else
            {
                // generates random index
                std::random_device rd;                              
                std::mt19937 gen(rd());                             
                std::uniform_int_distribution<> dist(0, PossibleMoves.size() - 1);  
                int randomIndex = dist(gen);

                op = curr;
                curr = PossibleMoves[randomIndex];
                pwr -= navigator_->GetDistanceToWaypoint(waypoints2[curr], op);
                pwr += waypoints2[curr].power;
                //OldMoves.push_back(op);
                Path.push_back(op);
            }
            
            // if finding first move in a valid path and path is valid so far -> nextPoint = curr
            if (count < 1)
            {
                nextPoint = curr;
                count++;
            }

            // if power is less than or equal to zero and the goal still hasn't been reached -> power and position are reset to initial values and will continue trying to find path
            if ((pwr <= 0) && (curr != goal))
            {
                pwr = currentPower_;
                curr = currWaypoint;
                count = 0;
                //OldMoves.clear();
                Path.clear();
                count2++;
            }
        }
    }
    //std::cout << "PanicSearch Iterations: " << count2 << std::endl;

    // whole path is returned to call in GetNextWaypoint once found and Path[1] will be returned to main.cpp 
    return Path;
}

// finds valid path and returns next move to main.cpp
std::string PathfinderMissionPlanner::GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint)
{
    std::vector<std::string> wp_names = navigator_->GetAllWaypointNames(); 
    std::map<std::string, Point2D> waypoints2;
    std::string curr = currentWaypoint;
    double dotprod;
    double angle;
    double pwr = currentPower_;
    std::vector<std::string> PossibleMoves;
    std::vector<std::string> OldMoves;
    std::vector<std::string> OldMoves2;
    std::vector<std::string> Path = {"START"};
    double closest = navigator_->GetDistanceToWaypoint(waypoints2[currentWaypoint], goalWaypoint);
    std::string nextPoint;
    std::string smallest;
    int count = 1;
    int count2 = 1;
    std::vector<std::string> panicPath;

    for (const auto& point : wp_names)
    { 
        waypoints2[point] = navigator_->GetWaypointPosition(point);
    } 

    // checks if the goal is a possible move and then returns goal as next move if true
    if (navigator_->GetDistanceToWaypoint(waypoints2[curr], goalWaypoint) <= pwr)
    {
        return goalWaypoint;   
    }

    // loops until valid path is found
    while (curr != goalWaypoint)
    {
        // clears possible moves vector after "changing" position
        PossibleMoves.clear();

        /* 
        PRINTS OLD MOVES
        std::cout << "OLD MOVES: ";
        for (const auto& point : OldMoves)
        {
             std::cout << point << " ";
        } 
        std::cout << std::endl;
        */

        // goes through all waypoints and picks out all possible moves
        for (const auto& moves : waypoints2)
        {
            //std::cout << "for loop entered" << std::endl;

            // sets current waypoints power to 0
            waypoints2[curr].power = 0;

            // finds angle between goal and a waypoint which is used to make condition impossible forcing PanicSearch call
            dotprod = ((waypoints2[goalWaypoint].y - waypoints2[curr].y) * (moves.second.y - waypoints2[curr].y)) + ((waypoints2[goalWaypoint].x - waypoints2[curr].x) * (moves.second.x - waypoints2[curr].x)) ;
            angle = acos(dotprod / (navigator_->GetDistanceToWaypoint(waypoints2[curr], goalWaypoint) * navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first)));

            //std::cout << "Angle Check: " << angle << std::endl;
            //std::cout << "Power required for " << moves.first << " " << navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) << std::endl;

            // if finding first move in path, possible moves can't be a past move in addition to being reachable and not the same as current position
            if (count == 1)
            {
                if ((navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) <= pwr) && (angle < (M_PI/1)) && (navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) != 0) && (std::find(OldMoves.begin(), OldMoves.end(), moves.first) == OldMoves.end()))
                {
                    //std::cout << moves.first << " PASS: " << angle << std::endl;
                    PossibleMoves.push_back(moves.first);
                }               
            }
            // if not finding first move in path, then the move can be a past move
            else if (count > 1)
            {
                if ((navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) <= pwr) && (angle < (M_PI/1)) && (navigator_->GetDistanceToWaypoint(waypoints2[curr], moves.first) != 0))
                {
                    //std::cout << moves.first << " PASS: " << angle << std::endl;
                    PossibleMoves.push_back(moves.first);
                }
            }
        }   

        /* 
        PRINTS POSSIBLE MOVES FOUND
        std::cout << "POSSIBLE MOVES: " << std::endl; 
          for (const auto& point : PossibleMoves)
        {
             std::cout << point << " ";
        } 
        std::cout << std::endl;
        */

        // if finding first move in path and possible moves is empty -> PanicSearch is called
        if (PossibleMoves.size() == 0 && count == 1)
        {
            std::cout << "\n[PATHFINDER] Impending Failure..... Panicking\n" << std::endl;
            panicPath = PanicSearch(currentWaypoint, goalWaypoint);
            return panicPath[1];  
        }

        // finds the move in possible moves that's closest to the goal
        for (const auto& moves : PossibleMoves)
        {
            //std::cout << "\"dist iterating  "<< moves << ": " << navigator_->GetDistanceToWaypoint(waypoints2[goalWaypoint], moves) << " closest: " << closest << std::endl;
            if (navigator_->GetDistanceToWaypoint(waypoints2[goalWaypoint], moves) < closest)
            {
                smallest = moves;
                closest = navigator_->GetDistanceToWaypoint(waypoints2[goalWaypoint], moves);
                
                //std::cout << "\"smallest reassign\": " << smallest << std::endl;
            }
        }
        closest = navigator_->GetDistanceToWaypoint(waypoints2[currentWaypoint], goalWaypoint);


        // if finding first move in path, then move is stored in nextPoint to be returned once full path found
        if (count == 1)
        {
            nextPoint = smallest;
        }

        // Infinite Loop Prevention
        if (count2 > 30)
        {
            std::cout << "INFINTE LOOP" << std::endl;
            return "";
        }

        // power adjusted and "current position" changed  and old position added to Path
        pwr -= navigator_->GetDistanceToWaypoint(waypoints2[curr], smallest);
        curr = smallest;
        count++;
        count2++;
        pwr += waypoints2[curr].power;
        //std::cout << "CURRENT POS: " << smallest << std::endl;
        Path.push_back(curr);
        //std::cout << "\"NEXT POINT\": " << nextPoint << std::endl;

        // if possible moves is zero but function isn't finding first move in path, everything is reset and the first move in the failed path is added to old moves
        // and function continues to try and find valid path
        if (PossibleMoves.size() == 0 && count > 1)
        {
            //std::cout << "RESET" << std::endl;
            OldMoves.push_back(nextPoint);
            nextPoint = "";
            pwr = currentPower_;
            curr = currentWaypoint;
            count = 1;
            Path.clear();

            for (const auto& point : wp_names)
            {
                waypoints2[point] = navigator_->GetWaypointPosition(point);
            } 
        }
    }   
    return nextPoint;
}