/**Recocido simulado para el problema de RBM*/
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
using namespace std;

mt19937_64 generador(0);

double uniforme(int n1, int n2){
  uniform_int_distribution<int> unif(n1, n2);
  return unif(generador);
}

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

/**Funcion para generar una solucion**/
void genera_sol(int **coor_rob, int cap[], int n, vector <vector <int>> &new_pos, int rob[], vector <int> &sol, int &dis){
  int p = 0, bx, by, mov = 0, mov_dos = 0, new_dis = 0;
  bx = new_pos[0][0]; //Posicion inicial de la bomba en el eje x
  by = new_pos[1][0]; //Posicion inicial de la bomba en el eje y
  dis = 0; //Importante reiniciar dis en 0
  for(int k = 0; k < n; k++){
    if(cap[rob[k]] > abs(coor_rob[rob[k]][0] - bx) + abs(coor_rob[rob[k]][1] - by)){ //Si el robot en la posicion k puede llegar a la posicion actual de la bomba
      mov_dos = mov;
      mov = cap[rob[k]] - abs(coor_rob[rob[k]][0] - bx) - abs(coor_rob[rob[k]][1] - by); //Movimiento (energia) disponible del robot en la pos. k despues de llegar a la bomba
      new_dis = mov + abs(bx - new_pos[0][0]) + abs(by - new_pos[1][0]); //Unidades de alejamiento (F.O.) que puede lograr el robot en la pos.
      if(new_dis > dis){
        p += 1;
        sol.push_back(rob[k]); //Nuevo robot en la solucion
        dis = new_dis; //Actualiza la (F.O.)
        new_pos[0].push_back(bx); //Nueva posicion de la bomba en el eje x
        new_pos[1].push_back(by); //Nueva posicion de la bomba en el eje y
      }else{
        mov = mov_dos; //Regresa a la capacidad de movimiento anterior
      }
    }
    bx = new_pos[0][p]; //Posicion actual de la bomba en el eje x
    by = new_pos[1][p]; //Posicion actual de la bomba en el eje Y
    if(k < n-1){
      calcula_pos(mov, bx, by, coor_rob[rob[k+1]][0], coor_rob[rob[k+1]][1]); //Funcion para calcular una nueva pos. para la bomba
    }else{
      /**Mueve la bomba a su posicion final con la energia sobrante del ultimo robot en la solucion**/
      int coor_x = 0, coor_y = 0;
      if(bx >= new_pos[0][0]){
        coor_x = 100000;
      }else{
        coor_x = -100000;
      }
      if(by >= new_pos[1][0]){
        coor_y = 100000;
      }else{
        coor_y = -100000;
      }
      calcula_pos(mov, bx, by, coor_x, coor_y);
      new_pos[0].push_back(bx);
      new_pos[1].push_back(by);
    }
  }
  return;
}

/**Funcion principal**/
int main(){
  auto t0 = std::chrono::high_resolution_clock::now(); //Inicio del cronometro
  int n, Bx, By;
  cin >> n >> Bx >> By; //n es el numero de robots
  int **coor_rob, cap[n]; //Coordenadas de los robots y capacidades de energia
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

  float prom_dis = 0;
  int corridas = 20;
  int observ[corridas];
  for(int iter = 0; iter < corridas; iter++){
    int dis = 0; //distancia de la pos inicial a la pos actual de la bomba(F.O.)
    vector <vector <int>> new_pos(2); //Matriz para las posiciones de la bomba en cada instante
    new_pos[0].push_back(Bx); //Posicion inicial de la bomba en x
    new_pos[1].push_back(By); //Posicion inicial de la bomba en y
    vector <int> num;

    /**Creación de numeros ordenados de 0 a n-1**/
    for(int i = 0; i < n; i++){
      num.push_back(i);
    }

    int pos, rob[n];
    /**Generación del vector que contiene todos los robots ordenados de manera aleatoria**/
    for(int i = 0; i < n; i++){
      pos = uniforme(0, num.size()-1);
      rob[i] = num[pos];
      num.erase(num.begin()+pos);
    }

    vector <int> sol; //Vector solucion que contiene la secuencia de los robots

    /**Calculo de la primer solucion y funcion objetivo**/
    genera_sol(coor_rob, cap, n, new_pos, rob, sol, dis); //Funcion para generar una solcuion a partir de un vector con todos los robots

    /**Asignacion de variables a comparar y reinicio de las iniciales*/
    int dis_fin = dis;
    vector <int> sol_fin = sol;
    vector <vector <int>> pos_fin(2);
    pos_fin[0] = new_pos[0];
    pos_fin[1] = new_pos[1];
    new_pos[0].erase(new_pos[0].begin()+1, new_pos[0].end());
    new_pos[1].erase(new_pos[1].begin()+1, new_pos[1].end());

    /**INICIA RECOCIDO SIMULADO**/
    float T = 100; //Temperatura inicial
    while(T >= 0.00001){ //Temperatura minima
      for(int k = 0; k < 10; k++){ //Numero de iteraciones con la temperatura actual
        for(int i = 0; i < n-1; i++){
          for(int j = i+1; j < n; j++){
            sol.clear(); //Se borran los elementos del vector para mandarlo vacio a la funcion genera_sol
            new_pos[0].erase(new_pos[0].begin()+1, new_pos[0].end()); //Se borran los elementos del vector (menos la posicion inicial) para mandarlo vacio a la funcion genera_sol
            new_pos[1].erase(new_pos[1].begin()+1, new_pos[1].end()); //Se borran los elementos del vector (menos la posicion inicial) para mandarlo vacio a la funcion genera_sol
            /**Intercambio de las posiciones del vector**/
            int guarda = rob[i];
            rob[i] = rob[j];
            rob[j] = guarda;
            genera_sol(coor_rob, cap, n, new_pos, rob, sol, dis); //Funcion para generar una solcuion a partir de un vector con todos los robots
            /**Verifica si mejora la funcion objetivo (F.O.)**/
            if(dis > dis_fin){ //Si la distancia (F.O.) mejora se actualiza la nueva solucion
              dis_fin = dis;
              sol_fin.clear();
              sol_fin = sol;
              pos_fin[0].clear();
              pos_fin[1].clear();
              pos_fin[0] = new_pos[0];
              pos_fin[1] = new_pos[1];
              //break; //Funciona mejor sin el break
            }else{
              /**Criterio de metropoli**/
              float pro = exp((dis - dis_fin)/T);
              float proba = uniforme(0, 10000)/10000.0;
              if(proba <= pro){ //Se puede aceptar una solucion peor con cierts probabilidad, si se acepta, se actualiza la nueva solucion
                dis_fin = dis;
                sol_fin.erase(sol_fin.begin(), sol_fin.end());
                sol_fin.clear();
                sol_fin = sol;
                pos_fin[0].clear();
                pos_fin[1].clear();
                pos_fin[0] = new_pos[0];
                pos_fin[1] = new_pos[1];
                //break; //Funciona mejor sin el break
              }else{ //En caso contrario se regresa al vector anterior
                rob[j] = rob[i];
                rob[i] = guarda;
              }
            }
          }
        }
      }
      float alpha = 0.1; //Parametro para el esquema de enfriamiento
      T -= T*alpha; //Se reduce la temperatura
    }
    /**TERMINA RECOCIDO SIMULADO**/

    /**Impresion de los resultados**/
    cout << "solucion numero " << iter+1 << endl;
    cout << "La solucion final es:\n";
    for(int i = 0; i < sol_fin.size(); i++){
      cout << sol_fin[i] << " ";
    }
    cout << "\nLos movimientos son:\n";
    for(int i = 1; i < pos_fin[0].size(); i++){ //Imprimo a pronposito desde i = 1 porque guarde dos veces la pos. inicial
      cout << pos_fin[0][i] << " " << pos_fin[1][i] << endl;
    }
    cout << "Distancia final: "<< dis_fin  << endl << endl;
    observ[iter] = dis_fin;
    prom_dis += dis_fin;
  }
  auto t1 = std::chrono::high_resolution_clock::now(); //Fin del cronometro

  /**Calculo del promedio, desviacion estandar, maximo y minimo de las ejecuciones del algoritmo**/
  prom_dis = prom_dis/corridas;
  float desv_est = 0;
  int maximo = 0, minimo = 100000000;
  for(int i = 0; i < corridas; i++){
    desv_est += (observ[i] - prom_dis)*(observ[i] - prom_dis);
    if(maximo < observ[i]){
      maximo = observ[i];
    }
    if(minimo > observ[i]){
      minimo = observ[i];
    }
  }
  desv_est = sqrt(desv_est/corridas);

  std::cout << "Duro " <<
                std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count() << " segundos\n";
  float prom_tiempo = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()/corridas;
  cout << "La distancia promedio es: " << prom_dis << endl;
  cout << "El tiempo promedio es: " << prom_tiempo << " segundos" << endl;
  cout << "La desviacion estandar es: " << desv_est << endl;
  cout << "El maximo es: " << maximo << endl;
  cout << "El minimo es: " << minimo << endl;
  return 0;
}
