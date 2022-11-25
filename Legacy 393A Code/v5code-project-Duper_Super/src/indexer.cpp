#include "indexer.h"
#include "vex.h"

//Helper Functions 
void setIndexer(int power) {
    indexer.spin(forward, power, volt);
}

//Driver Control Functions
void setIndexerMotors() {

    int indexerPower = 12.0 * (umisha.ButtonR1.pressing());
    setIndexer(indexerPower);
    
}