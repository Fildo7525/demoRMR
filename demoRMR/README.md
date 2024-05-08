# demoRMR
 
#  Zadanie 1

V prvom zadani sme spravili algoritmus na navadzanie robota na predefinovane body. Pouzili sme funkciu **setArcSpeed** na nastavenie rychlosti robota.
Nerobime to napriamo. Pouzivame nami vytvoreny ovladac (_RobotTrajectoryController_) na riadenie robota. Ten zaobstarava zrychlovanie robota po rampe
s preddefinovanym sklonom. Toto navadzanie je spravene neblokujuco. Pouzivame QT funkcionalitu signalov a slotov. A tada funguje to tak, ze najprv
emitneme signal s nastavenymi parametrami, ze chceme ist na bod so suradnicami (X, Y). Tieto suradnice vytvaraju usecku splu s nasou polohou.
Vygenerujeme preto sekvenciu bodov na tejto priamke a posielame ich do ovladaca. Ten ich postupne spracovava a posuva robota na tieto body.

Pri pohybe pouzivame P regulator s antiwindup zapojenim na rychlost a P regulator na otocenie. Regulator rychlosti ma proporcnu zlozku o velkosti 1000
a je obmedzeny hodnotami [-400mm/s; 400 mm/s]. Regulator otocenia ma proporcnu zlozku o veklosti 1. Nasledne sa spusti casovac, ktory zabezpecuje periodicke
upravovanie rychlosti a otocenia robota. Tento casovac ma nastaveny interval na 200ms. Pri zavolani slotu **on_positionTimerTimeout_changePosition**
sa vykonaju nasledovne veci

1. Vypocet chyby na zaklade aktualnej polohy a pozadovanej polohy
2. Vypocet akcneho zasahu pre rychlost a natocenie robota.
3. Emitnutie signalu na nastavenie rychlosti a natocenia robota po rampe.

Ako vypliva z tohoto postupu, cim blizsie sme ku koncovemu bodu tym pomalsie sa hybeme. Tento pohyb je ale ohraniceny na 15 mm/s. Tym poadom sa do ciela
dostaneme. Aby sme zahrnuli aj nejaku tu nepresnost zapocitali sme aj toleranciu na 5cm. Tym padom sa robot zastavi v okoli ciela. Tento sposob vie zabezpecit
ze sa dozastavime v okoli ciela aj keby nastal nejaky neziadany presmyk alebo ina neziaduca chyba.

# Zadanie 3

Pri zadani 3 bolo nasou ulohou mapovat prostredie, po ktorom sa robot pohybuje. Toto mapovanie sme realizovali pomocou rplidaru. Ten poskytuje data v podobe
laserovych merani, kde kazdy bod je reprezentovany jeho uhlom k robotu a jeho vzdialenostou. Ak lidar nezachytil ziadnu prekazku a teda nevygeneroval ziaden bod,
tak ma tento bod vzdialenost 0.

Na prepocet suradnic z polarnych na kartezianske sme pouzili vzorce:

#### Algoritmus na prepocet bodov lidaru do mapy
```cpp
for (size_t i = 0; i < numberOfScans; i += 2) {
	double distance = laserData.Data[i].scanDistance;
	double scanAngle = (360 - laserData.Data[i].scanAngle + 90) * TO_RADIANS;

	if (distance < m_robot->b) {
		continue;
	}

	double x = robotX + distance * std::sin(scanAngle + robotAngle);
	double y = robotY + distance * std::cos(scanAngle + robotAngle);

	points.append(QPointF(x, y) / 20);

	x /= TILE_SIZE;
	y /= TILE_SIZE;

	int mapX = x + m_map[0].size() / 4.;
	int mapY = y + m_map.size() / 2.;
}
```

V tom to algoritme sme vymenili sinus a cosinus pre x a y suradnice, pretoze sme mali inu orientaciu suradnicoveho systemu. Tym padom sme ziskali
spravne suradnice. To v koncovom dosledku pre nas znamenalo, ze vygenerovana mapa bola rovnako orientovana ako mapa v simulatore a v realite.

Tieto body sme vykreslovali do separatneho okna a zroven sme ich zapisovali aj do suboru na neskorisie pouzitie. Ako je znazornene v kode mapa ma
policka o nejakej velkosti. Pre nas bola velkost policka nastavena na velkost robota a teda to je 29cm.

# Zadanie 4

Pri zadani 4 sme pouzili mapu zo zadania 3 a poziciu zo zadania1. Vo vygenerovanej mape sme hladali cestu z pozicie robota do ciela. Tuto cestu sme
vytvarali pomocou algoritma **flood fill**.

Cely algoritmus fungoval nasledovne. Ked zadame do aplikacie cielovu poziciu, tak sa emitne signal do vedlajsieho vlakna, ktore sa stara o prepocitanie
trajektorie.

#### Algoritmus na prepocet trajektorie

```cpp
Preddefinovane: mapa prostreda (map), startovacia pozicia (startPoint),
		cielova pozicia (endPoint)
Vyuzivane: expandMap(map), floodFill(map, start, end), pruneTrajectory(trajektoria)
Vratene: trajektoria
---
funkcia plan(startPoint, endPoint)
	start = toMapCoordinates(startPoint)
	end = toMapCoordinates(endPoint)

	empa = expandMap(map)
	trajektoria = floodFill(emap, start, end)
	pruneTrajectory(trajektoria)

	return trajektoria
```

Najprv si vlozene data prepocitame z realnych suradnic (z metrov) do suradnic v mape (v polickach). Tym padom ziskame suradnice v mape.
Nasledne zavolame funkciu **expandMap**, ktora nam zvacsi steny v mape o velkost robota, t.j. o jedno policko. Tym padom sa vyhneme koliziam
so znamimi prekazkami. Nasledne zavolame funkciu **floodFill**, ktora nam vygeneruje cestu z pozicie robota do ciela. Tato cesta je vygenerovana
na zaklade 4 susednosti, ale body z vypocitaneho preistoru su brane z 8 susednosti. Tym padom sa vyhneme nejakym nezmyselnym zataceniam.

Nasledne aby sme ziskali cestu bez zbytocnych prejazdovych bodov zavolame funkciu **pruneTrajectory**. Tato funkcia vyberie z cesty len tie body,
ktore su nutne pre dosiahnutie ciela. Tym padom sa vyhneme zbytocnym zataceniam a prejazdovym bodom.

#### Algoritmus na flood fill

```python
function floodFill(start, end)
	markedMap = markMap(map, start, end)
	trajectory = pathFromMap(markedMap, start, end)
	return trajectory


function markMap(map, start, end)
	queue = []
	queue.push(start)

	while not queue.empty():
		curr = queue.pop()
		if curr == end:
			break;

		for i = 0 .. directions.size():
			tmp = curr + directions[i]
			if map[tmp] is not in walls:
				map[tmp] = map[curr] + 1
				queue.push(tmp)

	return map

function pathFromMap(map, start, end)
	trajectory = []
	curr = end
	trajectory.push_back(curr)

	while curr != start:
		trajectory.push_back(lowest from curr neighbours)
		curr = trajectory.back()

	return trajectory
```

#### Algoritmus na orezanie trajektorie

Preddefinovane: trajektoria

---
```python
Preddefinovane: path (trajektoria)
Vyuzivane: generatePoints(p1, p2, n) - vygeneruje n bodov na usecke p1, p2
	   isInCFree(point) - zisti ci je bod v Configuration free space
Vraciame: output
---
function pruneTrajectory(path)
	auto curr = path.begin();
	auto next = curr + 2;

	while next != path.end():
		output.push_back(*curr);
		midPoints = generatePoints(*curr, *next, 10);

		inCollision = false;
		for i = 0 .. midPoints.size():
			tmp = midPoints[i];
			if not in isInCFree(tmp):
				inCollision = true;
				break;

		if inCollision:
			curr = next - 1;
			next = curr + 2;
			continue;
		else:
			next++;
			if next is not path.end():
				output.remove_last();

	output.push_back(path.back());

	return output;
```

# Bonusove zadanie - Particle Filter

Pri publickovani dat z lidaru, ktore vyuziva robot na mapovanie prostreda, sme napojili dalsi slot na spracovanie dat.
Tieto data sa posielaju do particle filtra, ktory robi nasledovne:

1. Inicializuje castice v okoli pozicie robota,
2. Prepocita predchadzajuce data na vygenerovane castice,
3. Vypocita chybu pre kazdu casticu z prepocitanych dat na aktualne data z lidaru.
4. Vyberie casticu s najmensou odchylkou


