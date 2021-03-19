#include<list>
#include <RBE1001Lib.h>


enum TURN {TURN_RIGHT = -90, TURN_STRAIGHT = 0, TURN_LEFT = 90, TURN_UTURN = 180};

class Navigator
{
private: 
    enum ROAD {ROAD_A, ROAD_B, ROAD_C, ROAD_P, ROAD_P2, ROAD_1, ROAD_2, ROAD_3, ROAD_4};
    enum DEST {PICKUP, HOUSE_A, HOUSE_B, HOUSE_C};

    ROAD currRoad = ROAD_P; //start on H
    DEST currDest = PICKUP;
    //DEST prevDest = PICKUP;

public:
    ROAD GetRoad(void){return currRoad;}
    TURN CalcTurn(void); //calculates next turn; dependent on if path forward is blocked
    void updateDestination(int key);
    DEST GetDestination (void){return currDest; }
    bool DestHouseC(void){
        bool retVal = false;
        if (currDest == HOUSE_C){
            retVal = true;
        } return retVal;
    }
    bool PICKUPDestination(void)
    {
        bool value = false;
        if(currDest == PICKUP)
        {
            value = true;
        }
        return value;
    }
    bool ReachDestination(void)
    {
        bool value = false;
        switch(currDest){
            case PICKUP:
            if (currRoad == ROAD_P){
                value = true;
            }
            break;

            case HOUSE_A:
            if (currRoad == ROAD_A){
                value = true;
            }
            break;

            case HOUSE_B:
            if (currRoad == ROAD_B){
                value = true;
            } 
            break;            
            
            case HOUSE_C:
            if (currRoad == ROAD_C){
                value = true;
            }
            break;   

            default:
            break;         
        }
        return value;
    }
};