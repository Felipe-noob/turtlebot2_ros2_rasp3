#include "navigation.hpp"
#include <iostream>



std::vector<std::vector<double>> get_waypoints(uint32_t startID, uint32_t endID){

    std::vector<std::vector<double>> waypoints;
    GlobalMap map;

    switch (startID)
    {
    case INPUT_SIDE1:
        switch (endID)
        {
        case OUTPUT_SIDE1:
            std::cout << "Trajectory 1 to 3 " << '\n';

            /* Waypoints for a trajectory from WP1 to WP 3 */
            waypoints.push_back({map.WP1_1.x, map.WP1_1.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP3_1.x, map.WP3_1.y});
            waypoints.push_back({map.WP3.x, map.WP3.y});
                    
            break;
        case OUTPUT_SIDE2:
            std::cout << "Trajectory 1 to 4 " << '\n';

            /* Waypoints for a trajectory from WP1 to WP 4 */

            waypoints.push_back({map.WP1_1.x, map.WP1_1.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP4_1.x, map.WP4_1.y});
            waypoints.push_back({map.WP4.x, map.WP4.y});
            break;
            
        default:
            break;
        }
        /* code */
        break;
    
    case INPUT_SIDE2:
        switch (endID)
        {
        case OUTPUT_SIDE1:
            std::cout << "Trajectory 2 to 3 " << '\n';

            /* Waypoints for a trajectory from WP2 to WP 3 */
            waypoints.push_back({map.WP2_1.x, map.WP2_1.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP3_1.x, map.WP3_1.y});
            waypoints.push_back({map.WP3.x, map.WP3.y});
                    
            /* code */
            break;
        case OUTPUT_SIDE2:
            std::cout << "Trajectory 2 to 4 " << '\n';

            /* Waypoints for a trajectory from WP2 to WP 4 */

            waypoints.push_back({map.WP2_1.x, map.WP2_1.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP4_1.x, map.WP4_1.y});
            waypoints.push_back({map.WP4.x, map.WP4.y});

            break;
            
        default:
            break;
        }
        /* code */
    break;
    case OUTPUT_SIDE1:

        switch (endID)
        {
        case INPUT_SIDE1:
            std::cout << "Trajectory 3 to 1 " << '\n';

            /* Waypoints for a trajectory from WP3 to WP 1 */
            waypoints.push_back({map.WP3_1.x, map.WP3_1.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP1_1.x, map.WP1_1.y});
            waypoints.push_back({map.WP1.x, map.WP1.y});

            break;
        case INPUT_SIDE2:
            std::cout << "Trajectory 3 to 2 " << '\n';

            /* Waypoints for a trajectory from WP3 to WP 2 */
            waypoints.push_back({map.WP3_1.x, map.WP3_1.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP2_1.x, map.WP2_1.y});
            waypoints.push_back({map.WP2.x, map.WP2.y});

            break;
            
        default:
            break;
        }
       
    break;

    case OUTPUT_SIDE2:
        switch (endID)
        {
        case INPUT_SIDE1:
            std::cout << "Trajectory 4 to 1 " << '\n';

           /* Waypoints for a trajectory from WP4 to WP 1 */
            waypoints.push_back({map.WP4_1.x, map.WP4_1.y});
            waypoints.push_back({map.WP7.x, map.WP7.y});
            waypoints.push_back({map.WP6.x, map.WP6.y});
            waypoints.push_back({map.WP5.x, map.WP5.y});
            waypoints.push_back({map.WP1_1.x, map.WP1_1.y});
            waypoints.push_back({map.WP1.x, map.WP1.y});

            break;
        case INPUT_SIDE2:
            std::cout << "Trajectory 4 to 2 " << '\n';

           /* Waypoints for a trajectory from WP4 to WP 2 */
           waypoints.push_back({map.WP4_1.x, map.WP4_1.y});
           waypoints.push_back({map.WP7.x, map.WP7.y});
           waypoints.push_back({map.WP6.x, map.WP6.y});
           waypoints.push_back({map.WP5.x, map.WP5.y});
           waypoints.push_back({map.WP2_1.x, map.WP2_1.y});
           waypoints.push_back({map.WP2.x, map.WP2.y});

            break;
            
        default:
            break;
        }
    break;

    default:
        break;
    }

return waypoints;

} 