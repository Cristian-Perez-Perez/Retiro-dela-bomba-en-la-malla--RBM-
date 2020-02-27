/**Heuristica de grupos para el problema de RBM*/
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
using namespace std;

mt19937_64 generador(0);

/**Funcion para calcular la posicion**/
//Mueve de una unidad sobre el eje x y una unidad sobre el eje y a la vez
void calcula_pos(int mov, int &bx, int &by, int &rob_x, int &rob_y){ /**VERIFICAR SI ES NECESARIO ENVIAR LAS DIRECCIONES DE rob_x y rob_y**/
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

/**Funcion para calcular la energia en conjunto de cada grupo**/
void energia_grupal(vector <int> &ener_g, vector <vector <int>> &agrup, int cap[]){
  for(int i = 0; i < ener_g.size(); i++){
    for(int j = 0; j < agrup[i].size(); j++){
      ener_g[i] += cap[agrup[i][j]];
    }
  }
  return;
}

double uniforme(int n1, int n2){
  uniform_int_distribution<int> unif(n1, n2);
  return unif(generador);
}

int main(){
  int n_grup = 5; /**Numero de grupos (parametro modificable)**/
  int n, B_xx, B_yy;
  cin >> n >> B_xx >> B_yy; //n es el numero de robots, Bxx y Byy son las posiciones iniciales de la bomba en los ejes x & y
  int **coor_rob, cap_2[n]; //Coordenadas de los robots, capacidad de energia y distancia de la pos inicial a la pos actual de la bomba(F.O.)
  coor_rob = new int *[n];
  for(int i = 0; i < n; i++){
    coor_rob[i] = new int[2];
  }

  for(int i = 0; i < n; i++){
    for(int j = 0; j < 2; j++){
      cin >> coor_rob[i][j];
    }
    cin >> cap_2[i];
  }

  auto t0 = std::chrono::high_resolution_clock::now(); //Inicio del cronometro
  float prom_dis = 0;
  int corridas = 20; //Numero de veces que se ejecuta la heuristica
  int observ[corridas];
  for(int iter = 0; iter < corridas; iter++){
    int cap[n];
    for(int i = 0; i < n; i++){
      cap[i] = cap_2[i];
    }
    vector <vector <int>> new_pos(2); //Matriz para las posiciones de la bomba en cada instante
    new_pos[0].push_back(B_xx); //Posicion inicial de la bomba en x
    new_pos[1].push_back(B_yy); //Posicion inicial de la bomba en y

    /**INICIA K-MEDIAS**/
    int m = 0;
    int num_cent[n_grup];
    vector <int> num;
    for(int i = 0; i < n; i++){
      num.push_back(i);
    }
    /**Determinacion de los centroides iniciales**/
    //Los centroides se asignan tomando un robot por grupo de forma aleatoria
    while(m < n_grup){
      int al = uniforme(0, num.size()-1);
      num_cent[m] = num[al];
      num.erase(num.begin()+al);mt19937_64 generador(0);
      m++;
    }
    /**Asigna las coordenadas de los robots elegidos como centroides**/
    float cent[n_grup][2];
    for(int i = 0; i < n_grup; i++){
      cent[i][0] = coor_rob[num_cent[i]][0];
      cent[i][1] = coor_rob[num_cent[i]][1];
    }

    vector <vector<int>> agrup(n_grup);
    float guarda[n_grup][2];
    /**Modificacion de los grupos**/
    while(true){
      for(int i = 0; i < n; i++){
        int k = 0;
        float dis = 10000000.0;
        for(int j = 0; j < n_grup; j++){
          float new_dis = abs(coor_rob[i][0] - cent[j][0]) + abs(coor_rob[i][1] - cent[j][1]); //Calcula la distancia de cada uno de los robots a los centroides de cada grupo
          if(new_dis < dis){
            dis = new_dis;
            k = j;
          }
        }
        agrup[k].push_back(i); //Asigna el robot i al grupo k
      }
      /**Revisa si el numero de grupos se redujo**/
      for(int i = 0; i < n_grup; i++){
        if(agrup[i].size() == 0){
          agrup.erase(agrup.begin()+i);
          n_grup -= 1;
          i -= 1;
        }
      }
      /**Guarda las coordenadas de los centroides actuales**/
      for(int i = 0; i < n_grup; i++){
        guarda[i][0] = cent[i][0];
        guarda[i][1] = cent[i][1];
      }
      /**Calculo de los nuevos centroides**/
      //Calcula los nuevos centroides con los robots que hay en cada grupo
      for(int i = 0; i < n_grup; i++){
        float suma_x = 0, suma_y = 0;
        for(int j = 0; j < agrup[i].size(); j++){
          suma_x += coor_rob[agrup[i][j]][0];
          suma_y += coor_rob[agrup[i][j]][1];
        }
        cent[i][0] = suma_x/agrup[i].size();
        cent[i][1] = suma_y/agrup[i].size();
      }
      /**Comparativo de los centroides anteriores con los nuevos**/
      //Verifica si los grupos ya no cambiaron en la ultima iteracion, si es asi, se detiene el algoritmo
      int p = 0;
      for(int i = 0; i < n_grup; i++){
        if(guarda[i][0] != cent[i][0] || guarda[i][1] != cent[i][1]){ //Si los centroides cambiaron (entonces los grupos tambien)
          for(int q = 0; q < n_grup; q++){
            agrup[q].clear(); //Borra los grupos que se habian formado
          }
          break;
        }else if(i == n_grup-1){ //Esto quiere decir que el centroide ya no cambio (por tanto los grupos tampoco)
          p = 1;
        }
      }
      if(p == 1){
        break;
      }
    }
    /**TERMINA K-MEDIAS**/

    /**Calcula la energia grupal**/
    vector <int> ener_g;
    for(int i = 0; i < n_grup; i++){
      ener_g.push_back(0);
    }
    energia_grupal(ener_g, agrup, cap); //Funcion para sumar la energia los robots en cada grupo

    /**ordena los grupos por energia y sus respectivos indices de mayor a menor**/
    int ind[n_grup];
    for(int i = 0; i < n_grup; i++){
      ind[i] = i;
    }
    for(int i = 0; i < n_grup-1; i++){
      for(int j = i + 1; j < n_grup; j++){
        if(ener_g[i] < ener_g[j]){
          int save = ener_g[i];
          ener_g[i] = ener_g[j];
          ener_g[j] = save;
          int save_2 = ind[i];
          ind[i] = ind[j];
          ind[j] = save_2;
        }
      }
    }

    /**ALGORITMO PRINCIPAL PARA CALCULAR LA SOLUCION**/
    vector <int> sol; //Vector con la secuencia de robots de la solucion
    int mov = 0, dis = 0, q = 0;
    int B_x = new_pos[0][0], B_y = new_pos[1][0]; //Posiciones iniciales de la bomba en los ejes x & y
    for(int g = 0; g < n_grup; g++){
      int tam = sol.size(); //Tamaño de la solucion actual
      for(int k = 0; k < n_grup; k++){
        for(int i = 0; i < agrup[ind[k]].size(); i++){
          int new_mov = 0, p = 0; //p es un parametro de control
          int tam2 = sol.size();
          for(int j = 0; j < agrup[ind[k]].size(); j++){
            /**Para el primer robot en la solución**/
            if(tam2 == 0){
              //Movimiento (energia) disponible del robot j despues de llegar a la bomba (si es negativo no puede llegar)
              new_mov = cap[agrup[ind[k]][j]] - abs(coor_rob[agrup[ind[k]][j]][0] - B_x) - abs(coor_rob[agrup[ind[k]][j]][1] - B_y);
              if(cap[agrup[ind[k]][j]] - abs(coor_rob[agrup[ind[k]][j]][0] - B_x) - abs(coor_rob[agrup[ind[k]][j]][1] - B_y) > 0 && new_mov > mov){
                if(p == 0){
                  sol.push_back(agrup[ind[k]][j]);
                  p = 1;
                }else{
                  sol[q] = agrup[ind[k]][j];
                }
                mov = new_mov;
              }
              dis = mov; //Unidades de alejamiento (F.O.) que puede lograr el robot j del grupo k
              new_mov = mov;
            /**Para los demas robots en la solucion**/
            }else{
              B_x = new_pos[0][q-1];
              B_y = new_pos[1][q-1];
              calcula_pos(mov, B_x, B_y, coor_rob[agrup[ind[k]][j]][0], coor_rob[agrup[ind[k]][j]][1]); //Funcion para calcular una nueva pos. para la bomba
              /**Calcula si es posible que el robot llegue a la bomba con una pos. cercana**/
              if(cap[agrup[ind[k]][j]] - abs(coor_rob[agrup[ind[k]][j]][0] - B_x) - abs(coor_rob[agrup[ind[k]][j]][1] - B_y) > 0){
                //Movimiento (energia) disponible del robot j despues de llegar a la bomba (si es negativo no puede llegar)
                int new_mov2 = cap[agrup[ind[k]][j]] - abs(coor_rob[agrup[ind[k]][j]][0] - B_x) - abs(coor_rob[agrup[ind[k]][j]][1] - B_y);
                int new_dis = new_mov2 + abs(new_pos[0][0] - B_x) + abs(new_pos[1][0] - B_y); //Unidades de alejamiento (F.O.) que puede lograr el robot j del grupo k
                if(new_mov2 > new_mov && new_dis > dis){
                  new_mov = new_mov2;
                  dis = new_dis;
                  if(p == 0){
                    new_pos[0].push_back(B_x); //Nueva posicion de la bomba en el eje x
                    new_pos[1].push_back(B_y); //Nueva posicion de la bomba en el eje y
                    sol.push_back(agrup[ind[k]][j]); //Nuevo robot en la solucion
                    p = 1;
                  }else{
                    new_pos[0][q] = B_x; //Reemplaza la posicion actual de la bomba en el eje x
                    new_pos[1][q] = B_y; //Reemplaza la posicion actual de la bomba en el eje y
                    sol[q] = agrup[ind[k]][j]; //Reemplaza el ultimo robot en la solucion
                  }
                }
              }
            }
          }
          if(p == 1){
            q += 1;
          }
          if(tam2 == sol.size()){ //Verifica si el grupo k ya no puede aportar robots a la solucion
            break;
          }
          if(q > 0 && q > tam){
            mov = new_mov;
          }
          if(q > 0){
            cap[sol[q-1]] = 0; //Disminucion de la capacidad del robot elegido
          }
        }
        /**Actualiza la energia del grupo actual**/
        for(int z = 0; z < agrup[k].size(); z++){
          ener_g[k] += cap[agrup[k][z]];
        }
        /**Reordena los grupos con base en sus energias de mayor a menor**/
        for(int z = 0; z < n_grup-1; z++){
          for(int x = z + 1; x < n_grup; x++){
            if(ener_g[z] < ener_g[x]){
              int save = ener_g[z];
              ener_g[z] = ener_g[x];
              ener_g[x] = save;
              int save_2 = ind[z];
              ind[z] = ind[x];
              ind[x] = save_2;
            }
          }
        }
        if(sol.size() > tam){ //Si el grupo actual aporto robots se reinicia la secuencia de los grupos
          break;
        }
      }
      if(tam == sol.size()){ //Si ya ningun grupo aporto robots finaliza la revision de los grupos
        break;
      }
    }
    /**Mueve la bomba a su posicion final con la energia sobrante del ultimo robot**/
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
    /**TERMINA ALGORITMO PRINCIPAL**/

    /**Impresion de los resultados**/
    cout << "\nLa solucion final es:\n";
    for(int i = 0; i < sol.size(); i++){
      cout << sol[i] << " ";
    }
    cout << "\nLos movimientos son:\n";
    for(int i = 0; i < new_pos[0].size(); i++){
      cout << new_pos[0][i] << " " << new_pos[1][i] << endl;
    }
    cout << "Distancia final: "<< dis;
    observ[iter] = dis;
    prom_dis += dis;
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

  std::cout << "\nDuro " <<
                std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count( ) << " segundos\n";
  float prom_tiempo = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()/corridas;
  cout << "La distancia promedio es: " << prom_dis << endl;
  cout << "El tiempo promedio es: " << prom_tiempo << " segundos" << endl;
  cout << "La desviacion estandar es: " << desv_est << endl;
  cout << "El maximo es: " << maximo << endl;
  cout << "El minimo es: " << minimo << endl;
  return 0;
}

