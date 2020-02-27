/**Heuristica glotona para el problema de RBM*/
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
using namespace std;

/**Funcion para calcular la posicion**/
//Mueve de una unidad sobre el eje x y una unidad sobre el eje y a la vez
void calcula_pos(int mov, int &bx, int &by, int &rob_x, int &rob_y){
  while(mov > 0){
    if(bx == rob_x && by == rob_y){
      break;
    }
    if(bx - rob_x < 0){
      bx += 1;
      mov -= 1;
    }else if(bx - rob_x > 0){
      bx -= 1;
      mov -= 1;
    }
    if(by - rob_y < 0 && mov > 0){
      by += 1;
      mov -= 1;
    }else if(by - rob_y > 0 && mov > 0){
      by -= 1;
      mov -= 1;
    }
  }
  return;
}

int main(){
  int n, B_x, B_y;
  vector <vector <int>> new_pos(2); //Matriz para las posiciones de la bomba en cada instante
  cin >> n >> B_x >> B_y; //n es el numero de robots
  new_pos[0].push_back(B_x); //Posicion inicial de la bomba en x
  new_pos[1].push_back(B_y); //Posicion inicial de la bomba en y
  int **coor_rob, cap[n]; //Coordenadas y capacidad de energia de los robots
  coor_rob = new int *[n];
  for(int i = 0; i < n; i++){
    coor_rob[i] = new int[2];
  }

  for(int i = 0; i < n; i++){
    for(int j = 0; j < 2; j++){
      cin >> coor_rob[i][j];
    }
    cin >> cap[i];
  }

  auto t0 = std::chrono::high_resolution_clock::now(); //Inicio del cronometro
  vector <int> sol; //Vector que contiene a los robots de la solucion
  int mov = 0, dis = 0; //mov. disponible del robot despues de llegar a la pos. de la bomba y distancia de la pos. inicial a la pos. actual de la bomba(F.O.)

  /**COMIENZA HEURISTICA GLOTONA**/
  for(int i = 0; i < n; i++){
    int new_mov = 0;
    int tam = sol.size();
    int p = 0; //Parametro de control
    for(int j = 0; j < n; j++){
      if(i > 0){
        B_x = new_pos[0][i-1];
        B_y = new_pos[1][i-1];
        calcula_pos(mov, B_x, B_y, coor_rob[j][0], coor_rob[j][1]); //Funcion para calcular una nueva pos. para la bomba
      }
      /**Calcula si es posible que el robot llegue a la bomba con una pos. cercana**/
      int new_mov2 = cap[j] - abs(coor_rob[j][0] - B_x) - abs(coor_rob[j][1] - B_y); //Movimiento (energia) disponible del robot j despues de llegar a la bomba (si es negativo no puede llegar)
      int new_dis = new_mov2 + abs(new_pos[0][0] - B_x) + abs(new_pos[1][0] - B_y); //Unidades de alejamiento (F.O.) que puede lograr el robot j
      /**Asignación del robot con mayor cap. de movimiento y que aleja más la bomba**/
      if(new_mov2 > new_mov && new_dis > dis){
        new_mov = new_mov2;
        dis = new_dis;
        if(p == 0){
          if(i > 0){
            new_pos[0].push_back(B_x); //Nueva posicion de la bomba en el eje x
            new_pos[1].push_back(B_y); //Nueva posicion de la bomba en el eje y
          }
          sol.push_back(j); //Nuevo robot en la solucion
          p = 1;
        }else{
          new_pos[0][i] = B_x; //Reemplaza la posicion actual de la bomba en el eje x
          new_pos[1][i] = B_y; //Reemplaza la posicion actual de la bomba en el eje y
          sol[i] = j; //Reemplaza el ultimo robot en la solucion
        }
      }
    }
    if(i == sol.size()-1){
      mov = new_mov; //Asigna el movimiento (energia) disponible del robot elegido en i
    }
    if(tam == sol.size()){ //Condicion para ver si no hubo mas robots que entraran en la solucion
      break;
    }
    cap[sol[i]] = 0; //La energia del robot elegido en i pasa a ser 0
  }
  /**Mueve la bomba a su posicion final con la energia sobrante del ultimo robot en la solucion**/
  int coor_x = 0, coor_y = 0, bx, by;
  bx = new_pos[0][sol.size()-1]; //Penultima posicion de la bomba en x
  by = new_pos[1][sol.size()-1]; //Penultima posicion de la bomba en y
  if(bx >= new_pos[0][0]){
    coor_x = 1000000;
  }else{
    coor_x = -1000000;
  }
  if(by >= new_pos[1][0]){
    coor_y = 1000000;
  }else{
    coor_y = -1000000;
  }
  calcula_pos(mov, bx, by, coor_x, coor_y); //Calcula la posicion final de la bomba
  new_pos[0].push_back(bx);
  new_pos[1].push_back(by);
  /**TERMINA HEURISTICA GLOTONA**/

  /**Impresion de los resultados**/
  cout << "La solucion final es:\n";
  for(int i = 0; i < sol.size(); i++){
    cout << sol[i] << " ";
  }
  cout << "\nLos movimientos son:\n";
  for(int i = 0; i < new_pos[0].size(); i++){
    cout << new_pos[0][i] << " " << new_pos[1][i] << endl;
  }
  cout << "Distancia final: "<< dis;
  auto t1 = std::chrono::high_resolution_clock::now(); //Fin del cronometro
  std::cout << "\nDuro " <<
                std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count( ) << " segundos\n";
  return 0;
}
