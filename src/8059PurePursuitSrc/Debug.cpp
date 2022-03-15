#include "main.h"
void printVector(std::vector<double> vd){
  for(auto m : vd){
    printf("%.2f ", m);
  }
  printf("\n");
}
void posPrintTerminal(){
  printf("X: %.2f Y: %.2f bearing: %.2f\n", position.getX(), position.getY(), bearing*toDeg);
}
void posPrintMaster(){
  Controller master(E_CONTROLLER_MASTER);
  master.print(2, 0, "%.2f %.2f %.2f", position.getX(), position.getY(), bearing*toDeg);
}
void encdPrintTerminal(){
  printf("EncdS: %4.1f \t EncdR: %4.1f\n", encdS, encdR);
}

void Debug(void * ignore) {
  while(true) {
    switch(DEBUG_MODE) {
      case 0:
        posPrintTerminal();
        break;
      case 1:
        encdPrintTerminal();
        break;
    }
    posPrintMaster();

    delay(dT);
  }
}
