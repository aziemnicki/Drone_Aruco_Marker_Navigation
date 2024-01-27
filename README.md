# ARL_Tello_and_Aruco
## Temat projektu: 
Budowa mapy otoczenia przy użyciu znaczników ArUco. **bold text**
## Założenia 
- Budowa przestrzeni laboratorium, w którym będzie poruszał się dron.
- Zbudowanie trasy z naklejonych znaczników AruCo i przypisanie im cech pozwalających na lokalizację jednostki latającej.
- Odnajdywanie znaczników zgodnie z poleceniami wysyłanymi z komputera. 
- Dolatywanie do punktów wpisanych w terminalu.
## Zastosowane technologie
Projekt realizowano w kontenerze Docker, który utowrzony został na podstawie Dockerfile pomagającym w zainstalowaniu potrzebnych bibliotek. Główne biblioteki wykorzystywane w projecie to:
- ROS2 foxy
- transformations: przekształcenia  np: kątów Eulera
- [tello_ros](https://github.com/clydemcqueen/tello_ros)
- [fiducial_vlam](https://github.com/ptrmu/fiducial_vlam)
- [ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco): wykrywanie znaczników ArUco
- threading: wykorzystanie timerów do cyklicznego wywoływania funkcji i jednoczesnego odczytu danych np: z Gazebo lub Optitracka

Dodatkowo w projekcie napisana została klasa PID, w której odbywa się przeliczanie błędów pozycji oraz kątów jak i resetowanie współczynnika całkowania po doleceniu do punktu.
Symulacja odbywa się w programie Gazebo. 

## Podział i organizacja prac
### Andrzej Ziemnicki 
>Stworzenie świata symulacji, kontenera Docker, umożliwienie odczytywania pozycji drona w Gazebo i z OptiTrack'a, utrzymanie repozytorium.

### Norbert Mostowski
>Implementacja logiki działania programu i maszyny stanów, utworzenie misji dolatywania do punktów oraz logiki ścieżki powrotnej.

### Jeremiasz Wojciak
>Implementacja tworzenia ścieżki z punktów, odczytywania danych z markerów oraz wprowazdania wyboru punktów. 

Każde zmiany były na bieżąco konsultowane oraz wprowadzane zgodnie z zasadami CI/CD tak, aby kod ulegał ciągłej optymalizacji i rozbudowania funkcji.

## Wygląd środowiska w symulacji


## Zakres zrealizowanych prac w symulacji
Głównymi celami projektu było stworzenie działającej symulacji z możliwością zadawania misji dolecenia do zadanego punktu oraz powrotu do pozycji "home". Zrealizowano następujące cele:
|      Gazebo            | Opis prac |
| ----------- | ----------   |
| 1. Odwzorowanie laboratorium w symulacji i utworzenie w niej trasy z nałożonymi markerami.| Wykonan pomiary laboratorium, za pomocą edytora modelów w Gazeboo ustawiono ściany pomieszczenia. |
| 2. Stworzenie logiki bazującej na maszynie stanów | W klasie TelloState utowrzono funkcje, które informują o aktualnie wykonywanym kroku programu. Główną funkcją jest contorller, który zarządza logiką działania.  |
| 3. Odczytywanie pozycji drona z Gazebo. | W pliku konfiguracyjny świata dodano plugin obsługujący wysyłanie do Node'ów ROSa pozycji drona. |
| 4. Dodanie timerów do cyklicznego wysyłania prędkości. | W każdej powtarzalnej funkcji programu dodano Timer wykonujący się z częstotliwościa 10 Hz. tak jak odczyt pozycji z Gazebo |
| 5. Napisanie własnej klasy regulatora PID oraz dobranie nastaw. | Z uwagi na brak niektórych funkcji w simple_pid utworzono własną klasę regulatora. |
| 6. Implementacja resetowania uchybu dla obrotu przy przekraczaniu wartości 2pi/-2pi. | Zapewnia możliwość dokłądnego obrotu o kąt z zadaną precyzją. |
| 7. Napisanie subscribera do odczytywania znaczników przy użyciu biblioteki ros2_aruco. | Wykrycie znacznika odbywa się tylko raz, jeśli dron wykryje go w odległości nie większej niż 0.5m od siebie aby zminimalizować błąd wykrycia 2 znaczników naraz. |
| 8. Możliwość zadania trasy do wykonania złożonej z wielu punktów. | Kolejne pozycje lotu dodawane są na podstawie odczytanych względnych wartości ze znaczników ArUco. |
| 9. Utworzenie misji osiągania zadanej pozycji z określoną dokładnością. | Realizuje to funkcja mission_function. Prezycja ustawiana jest na podstawie odległości euklidesowej.|
| 10. Budowa trasy powrotnej, powrót do punktu startowego. | Utworzenie trasy powrotnej z zapisanych punktów odbywa się poprzez odwrócenie ich kolejności z pominięciem punktów znajdujących sie w tej samej osi. |
| 11. Zadanie z terminala i dolecenie do wybranego punktu. | Odczytanie numeru znacznika ArUco podanego w terminalu jako input. |
| 12. Powrót do punktu startowego i oczekiwanie na dalsze instrukcje. | Ponowne wywołanie funkcji return z punktami z poprzedniej ścieżki. |

## Instrukcja uruchomienia

Domyślnie modele znaczników ArUco w środowisku Gazebo są niewidoczne przez drona, ponieważ ścieżka do ich dostępu jest w nieodpowiednim folderze.
Aby dron widział znaczniki w swojej kamerze i mógł je rozpoznawać należy przenieść znaczniki do folderu `root/.gazebo/models`. 
Dodatkowo, aby pobierać aktualną pozycje drona w Gazebo należy do modelu świata (plik  `NAZWA.world`) dodać plugin:
```
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
        <namespace>/gazebo</namespace>
    </ros>
    <!-- <update_rate>0.1</update_rate> -->
</plugin>`
``````
Powyższy plugin dodajemy zaraz za linią `<world name='default'>:`

Aby obsługiwać symulację konieczne jest uruchomienie wizualizacji
> xhost +local:root

Po utworzeniu kontenera za pomocą Dockerfile'a należy przejść do katalogu `tello_ros_ws` oraz przebudowanie projektu
> cd tello_ros_ws<br>
> source /opt/ros/foxy/setup.bash<br>
> colcon build --symlink-install<br>
> source install/setup.bash <br>

Następnie uruchomienie symulacji musi odbywać się w poniższej kolejności
### 1. Uruchomienie środowiska
> ros2 launch tello_gazebo simple_launch.py
### 2. Uruchomienie Node'a controllera i oczekiwanie za zadanie lotu
> ros2 run tello_controller controller_gazebo
### 3. Uruchomienie Node'a wykrywającego znaczniki ArUco
> ros2 launch ros2_aruco aruco_recognition.launch.py
### 4. Publikacja topica startującego drona
> ros2 topic pub --once /iisrl/tello_controller std_msgs/msg/Empty

Po przeleceniu całej trasy i zapisania punktów znaczników uruchomi się w oknie controllera polecenie zadania punktu, do którego dron ma dolecieć. Wystarczy wpisać numer od 1 do 7 (ilość znaczników).

Opcjonalnie można wyświetlić topic, na który wysyłane są pozycje i ID markerów ArUco po ihc wykryciu. 
> ros2 topic echo /aruco_markers

Można także w `rviz2` wyświetlić podgląd obrazu z kamery drona wyświetlając topic `/drone1/camera_info`


## Instrukcja uruchomienia w laboratorium 
Aby uruchomić program na rzeczywistym dronie w laboratorium należy na początku połączyć się z nim poprzez WIFI a następnie:
### 1. Uruchomić Node'a obsługującego odczyt danych z OptiTrack'a
> ros2 run optitrack optitrack_node

### 2. Uruchomić przesyłanie komend za pomocą teleop
> ros2 launch tello_driver teleop_launch.py

Dron jest gotowy do wystartowania, w momencie, gdy na ekranie ukaże się podgląd z kamery w osobnym oknie

### 3. Wykonać kroki 2-4 z uruchomienia symulacji w Gazebo

### 4. W przpadku konieczności lądowania należy wysłać w osobnym terminalu polecenie
> ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"


## Efekt prac w Gazebo



## Zakres zrealizowanych prac w laboratorium
W laboratorium przetestowano powższe rozwiązania i dodano także następujące modyfikacje

|      Laboratorium            | Opis prac |
| ----------- | ---------------|
| 1. Odczytywanie i przetwarzanie danych z OptiTrack’a. | Zmiana źródła danych o pozycji i orientacji. |
| 2. Ograniczanie prędkości w zakresie (-15, -6) i (6, 15). | Względy bezpieczeństwa pracy na rzeczywistym sprzęcie (dron DJI Tello) w laboratorium. |
| 3. Brak wykorzystywania osi Z - lot na jednej płaszczyźnie. | Brak zadawania sterowania do lotu w osi Z (góra- dół). |
| 4. Inne nastawy regulatora PID oraz resetowanie członu całkującego po osiągnięciu zadanej pozycji. | Redukcja uchybu oraz resetowanie zapisanch wartości całkowania. |
| 5. Uwzględnienie orientacji odczytanych pozycji z OptiTracka’a. | Układ globalny został przeliczony do układu lokalnego drona, ponieważ osie x oraz y są zamienione znakami.  |
| 6. Zadanie rosnącej sekwencji id znaczników AruCo. | Zmniejszenie prawdopodobieństwa odczytania złego znacznika ArUco podczas loty. Wymagane z uwagi na bardzo duży zasięg odczytywania znaczników (do 4 metrów względem drona). |
| 7. Zmiana dokładności osiąganej pozycji. | W rzeczywistości występuje dryf drona, który unimożliwia osiąganie pozycjiz dokładnością do 1 cm lub 1 sstopnia obrotu. |

## Efekt prac w laboratorium


## Możliwości rozwoju
W dalszej części rozwoju projektu można przetestować:
* empiryczny dobór bardziej optymalnych nastaw dla rzeczywistego drona 
* przetestowanie programu z większymi prędkościami lotu
* optymalizacja lotu z większą precyzją pozycjonowania oraz mniejszymi oscylacjami wokól zadanych pozycji
* minimalizacja czasu postoju podczas dolecenia do punktu
* modernizacja lub stworzenie nowej, bardziej skomplikowanej trasy
* dodanie przelotu na różnych wysokościach znaczników
* dodanie zabezpieczeń np: wpisania nieodpowiedniego numeru markera, zbyt dalekiego odlatywania od punktu czy też zachowania w przypadku utraty widoczności kamery bądź OptiTrack'a.