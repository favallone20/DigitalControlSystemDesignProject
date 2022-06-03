# Progetto Digital Control System Design
## Controllo di un motore DC con retroazione di stato
***

* Francesco Avallone <f.avallone20@studenti.unisa.it>
* Lorenzo Pagliara <l.pagliara5@studenti.unisa.it>

***
### Organizzazione cartella
La cartella del progetto è organizzata nel modo seguente:

```
Progetto_CCAR
└─── Position_State_Feedback_MATLAB_Simulink
│   └─── Position_State_Feedback.mlx
|   └─── Controller_Observer_Model.slx
|   └─── Position_State_Feedback_Simulation.slx
|   └─── Position_State_Feedback_SIL_Simulation.slx
|   └─── Position_State_Feedback_PIL_Simulation.slx
|   └─── Position_State_Feedback_Motor.slx
└─── Speed_State_Feedback_MATLAB_Simulink
│   └─── Speed_State_Feedback.mlx
|   └─── Controller_Observer_Model.slx
|   └─── Speed_State_Feedback_Simulation.slx
|   └─── Speed_State_Feedback_SIL_Simulation.slx
|   └─── Speed_State_Feedback_PIL_Simulation.slx
|   └─── Speed_State_Feedback_Motor.slx 
└─── Position_State_Feedback_Direct_Coding
│   └─── ...
└─── Speed_State_Feedback_Direct_Coding
│   └─── ...
└─── Report.pdf
└─── README.md
```

|File|Descrizione|
|:---|:---| 
|`Position_State_Feedback.mlx`| É il file principale per il calcolo dei guadagni del controllore e per l'esecuzione di tutte le simulazioni del controllo in posizione.|
|`Speed_State_Feedback.mlx`| É il file principale per il calcolo dei guadagni del controllore e per l'esecuzione di tutte le simulazioni del controllo in velocità.|
|`Controller_Observer_Model.slx`| É il file contenente il modello state feedback controller e osservatore.|
|`Position_State_Feedback_Simulation.slx`| É il file contenente lo schema per l'esecuzione della simulazione MIL del controllo in posizione.|
|`Speed_State_Feedback_Simulation.slx`| É il file contenente lo schema per l'esecuzione della simulazione MIL del controllo in velocità.|
|`Position_State_Feedback_SIL_Simulation.slx`| É il file contenente lo schema per l'esecuzione della simulazione SIL del controllo in posizione.|
|`Speed_State_Feedback_SIL_Simulation.slx`| É il file contenente lo schema per l'esecuzione della simulazione SIL del controllo in velocità.|
|`Position_State_Feedback_PIL_Simulation.slx`| É il file contenente lo schema per l'esecuzione della simulazione PIL del controllo in posizione.|
|`Speed_State_Feedback_PIL_Simulation.slx`| É il file contenente lo schema per l'esecuzione della simulazione PIL del controllo in velocità.|
|`Position_State_Feedback_Motor.slx`| É il file contenente lo schema per l'esecuzione sul mototre fisico del controllo in posizione.|
|`Speed_State_Feedback_Motor.slx`| É il file contenente lo schema per l'esecuzione sul mototre fisico del controllo in velocità.|

***

### Guida agli schemi Simulink
Il controllore digitale e l'osservatore di Luenberger, componenti necessarie per effettuare il controllo in retroazione di stato del motore, che sia esso in posizione o in velocità, sono stati implementati nello schema Simulink `Controller_Observer_Model.slx`. Per una più semplice comprensione dello schema, i vari elementi che compongono i due componenti, sono stati raggruppati in blocchetti logici.

Lo schema ottenuto è stato poi importato in tutti gli altri schemi di simulazione mediante il blocchetto Simulink *Model* e collegato al modello matematico del motore o interfacciato direttamente con il processo fisico.

Lo script principale (`Position_State_Feedback.mlx`/`Speed_State_Feedback.mlx`), che può essere avviato mediante il pulsante:

```
Run
```
è stato implementato in maniera tale da poter aprire gli schemi Simulink in maniera automatica ed effettuare la simulazione MIL e mostrarne i risultati senza che l'utente faccia nient'altro. Per quanto riguarda tutte le altre simulazioni, una volta che gli schemi verranno aperti in maniera automatica dallo script, sarà compito dell'utente avviare la simulazione SIL/PIL e l'esecuzione sul motore.

**Attenzione**
Per l'esecuzione delle simulazioni SIL, PIL e del codice sul motore modificare opportunamente la porta COM impostata sugli schemi.
***

### Note

1. Le versioni Matlab e Simulink utilizzate sono:

    * MATLAB  Version 9.11 (R2021b);
    * Simulink Version 10.4 (R2021b).
2. La versione di STM32CubeIde utilizzata è la 1.8.0.

