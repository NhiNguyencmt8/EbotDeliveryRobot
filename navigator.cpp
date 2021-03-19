#include <navigator.h>
#include <list>
#include <RBE1001Lib.h>



extern Rangefinder ultrasonic;


void Navigator::updateDestination (int key){
  if(key == 16){
    currDest = HOUSE_A;
  } else if (key == 17){
    currDest = HOUSE_B;
  } else if (key == 18){
    currDest = HOUSE_C;
  } else if (key == 20){
    currDest = PICKUP;
  }
}

TURN Navigator::CalcTurn(void)  // returns the angle (in degrees) to turn
{   
    switch(currRoad)
    {
      case ROAD_P:// To the intersection at the PickUP zone
        if (currDest == PICKUP)
        {
          currRoad = ROAD_P;
          //return TURN_STRAIGHT;
        } 
        if (currDest != PICKUP) // checking for blocking contr  ultrasonic.getDistanceCM() < 20
        {
          Serial.println("Distance is ");
          Serial.println(ultrasonic.getDistanceCM());
          if (ultrasonic.getDistanceCM() > 20)
          {
            currRoad = ROAD_3;
            return TURN_STRAIGHT;
          }

            else 
            {
                currRoad = ROAD_2;
                return TURN_RIGHT;
            }
         
        }

        break;



      case ROAD_2: // 1st intersection to 4th intersection
       if(currDest == HOUSE_C || currDest == HOUSE_A || currDest == HOUSE_B){
         currRoad = ROAD_1;
         return TURN_LEFT;
       } else if (currDest == PICKUP){
         currRoad = ROAD_P;
         return TURN_LEFT ;
       }
        break;
      case ROAD_1: // 4th intersection to 3rd intersection
        if(currDest == HOUSE_C)
        {
          currRoad = ROAD_C;
          return TURN_RIGHT;
        } 
        else if (currDest == HOUSE_A || currDest == HOUSE_B)
        {
          currRoad = ROAD_4;
          return TURN_LEFT;
        } 
        else if (currDest == PICKUP)
        {
          currRoad = ROAD_2;
          return TURN_RIGHT;
        }

        break;
      case ROAD_3: 
      if(currDest == PICKUP)
      {
        currRoad = ROAD_P;
        return TURN_STRAIGHT;
      } 
      else if (currDest == HOUSE_A)
      {
        currRoad = ROAD_A;
        return TURN_LEFT;
      } 
      else if (currDest == HOUSE_B)
      {
        currRoad = ROAD_B;
        return TURN_STRAIGHT;
      } 
      else if (currDest == HOUSE_C)
      {
        currRoad = ROAD_4;
        return TURN_RIGHT;
      }
        break;

      case ROAD_4: // 3rd intersection to 2nd intersection
      if(currDest == HOUSE_B){
        currRoad = ROAD_B;
        return TURN_RIGHT;
      } else if (currDest == HOUSE_A){
        currRoad = ROAD_A;
        return TURN_STRAIGHT;
      } else if (currDest == HOUSE_C ){
        currRoad = ROAD_C;
        return TURN_STRAIGHT;
      } else if ( currDest == PICKUP ){
        currRoad = ROAD_1; // CHECK BACK!!
        return TURN_RIGHT ;
      }
        break;

      case ROAD_A: 
        if (currDest == PICKUP){
          currRoad = ROAD_3;
          return TURN_RIGHT;
        }
        if (currDest == HOUSE_A){
          currRoad = ROAD_A;
        }
        break;
      
      case ROAD_B: 
        if (currDest == PICKUP){
          currRoad = ROAD_3;
          return  TURN_STRAIGHT;
        }
        if (currDest == HOUSE_B){
          currRoad = ROAD_B;
        }
        break;      

       case ROAD_C: 
        if (currDest == PICKUP){
          currRoad = ROAD_1;
          return TURN_LEFT;
        }
        if (currDest == HOUSE_C){
          currRoad = ROAD_C;
        }
        break;       
        
      default:
        break;
  }
}
