### Funkcia: calculateOdometry

Táto funkcia je metódou `MainWindow` triedy zodpovedná za výpočet odometrie na základe poskytnutých dát `TKobukiData`. Vykonáva sa periodicky s každým spustením funckie procesThisRobot(). Úlohy funkcie:

1. **Výpočet diferencií z enkóderov**:
   - Výpočet diferencie v dátach z enkóderov (`diffLeftEnc` a `diffRightEnc`) medzi ich predošlou a aktuálnou hodnotou.

2. **Kontrola pretečenia enkóderov**:
   - Skontroluje sa pretečenie enkóderov (z minimálnej na maximálnu hodnotu alebo naopak). Oprava chyby o `SHORT_MAX` ak sa vyskytla. 

3. **Pričítanie prejdenej vzdialenosti**:
   - Výpočet prejdenej čiastkovej vzdialenosti každého z kolies na základe diferencií enkóderov a faktora konverzie (`robot.tickToMeter`).

4. **Update pomocných premenných**:
   - Aktualizácia predošlých hodnôt z enkóderov (`lastLeftEncoder` a `lastRightEncoder`) pre ďalšie výpočty.

5. **Update pozície robota**:
   - Aktualizácia polohy robota (`m_x`, `m_y`, a `m_fi`) na základe vypočítanej prejdenej vzdialenosti a uhla natočenia z gyroskopu.
   - Úprava počiatočnej polohy robota (`m_x`, `m_y`) ak nie je nastavená.

### Časovač m_positionTimer

Pretečenie tohto časovača spúšťa funkciu 'on_positionTimerTimeout_changePosition', ktorá je zodpovedná za ovládanie rýchlosti a natočenia robota na základe typu vykonávaného pohybu.

1. **Výpočty chýb na základe vykonávaného pohybu**:
   - Ak je typ pohybu 'Rotation', počítaná je chyba rotácie využitím funkcie 'localRotationError()'. Ak je aktuálny typ pohybu nastavený na 'Forward', počítaná je chyba prejdenej vzialenosti aj natočenia robota. Pri type pohybu 'Arc' je počítaná chyba prejdenej vzdialenosti. 


### Funkcia: rotateRobotTo

Táto funckia slúži na natočenie robota o definovaný uhol. Využívaná je pred spustením všetkých typov trajektórií. Rotačný pohyb vykonáva pomocou časovača 'm_positionTimer'.

### Plánovanie lineárnej trajektórie a trajektórie oblúka

1. **Handler na stlačenie tlačidiel**
    - Po zaklniknutí tlačidiel vygenerovania trajektórie príslušného typu sa aktualizujú cieľové súradnice 'm_xTarget' a 'm_yTarget' a spustí sa funkcia 'calculateTrajectory', ktorá obsahuje nasledovné funkcie:
2. **calculateTrajectoryTo**
    - Výpočet vzdialenosti a natočenia robota voči želanému bodu. Na výpočet natočenia sa využíva funkcia 'atan2()' a na výpočet vzdialenosti Euklidov vzorec.
3. **computeLineParameters**
    - Výpočet parametrov priamky na základe dvoch poskytnutých bodov na nej ležiacich. Využíva sa na výpočet rovnice priamky so súradnicami robota a želaného cieľového bodu.
4. **generateSequence**
    - Vygenerovanie sekvencie bodov pozdĺž poskytnutej čiary s definovaným počiatočným a koncovým bodom. Využíva sa na diskrétnu definíciu lineárnej trasy robota.
5. **handleLineResults**
    - Funkcia spustená v prípade vyžiadaného lineárneho pohybu. Nastavia sa vypočítané plánované waypointy a spustí sa funckia pre natočenie robota na potrebný uhol('rotateRobotTo'). Pohyb je následne spustený zapnutím časovača 'positionTimer', ktorého funckia sa vykonáva periodicky.
6. **handleArcResults**
    - Funkcia spustená v prípade vyžiadaného pohybu po krivke. Nastavia sa vypočítané plánované waypointy, spustí sa funckia pre natočenie robota na potrebný uhol('rotateRobotTo') a inicializuje sa  PID regulátor pre rýchlosť a natočenie robota počas vykonávanie pohybu. Pohyb je následne spustený zapnutím časovača 'positionTimer', ktorého funckia sa vykonáva periodicky.








