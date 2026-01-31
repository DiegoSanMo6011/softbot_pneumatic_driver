# Diagrama neumatico (version inicial basada en diagrama actual)

```mermaid
flowchart LR
    subgraph Presion["Linea de presion"]
        P1["Bomba P1"] --> V32P1["Valvula 3/2 NC (P1)"]
        P2["Bomba P2"] --> V32P2["Valvula 3/2 NC (P2)"]
        V32P1 --> PAR["Linea PAR"]
        V32P2 --> PAR
        PAR --> HDRSEL["HDR_IN_SEL"]
    end

    subgraph Vacio["Linea de vacio"]
        V1["Bomba V1"] --> V32V1["Valvula 3/2 NC (V1)"]
        V2["Bomba V2"] --> V32V2["Valvula 3/2 NC (V2)"]
        V32V1 --> VAC["Linea VAC"]
        V32V2 --> VAC
    end

    HDRSEL --> HDR["HEADER / MANIFOLD"]
    VAC --> HDR

    HDR --> CONTROL["CONTROL (linea comun)"]

    CONTROL --> TEEBST["TEE_BST (BOOST)"]
    CONTROL --> TEESNS["TEE_SNS (SENSOR)"]
    CONTROL --> TEEAT["TEE_AT (Anclaje trasero)"]
    CONTROL --> TEEAD["TEE_AD (Anclaje delantero)"]

    TEEBST --> CHECK_FILL["CHECK (sistema->tanque)"] --> T[(Tanque metal)]
    T --> CHECK_OUT["CHECK (tanque->sistema)"] --> V22BST["Valvula 2/2 NC (BOOST)"] --> TEEBST
    TEESNS --> S["Sensor presion"]
    TEEAT --> V32AT["Valvula 3/2 NC (AT)"] --> AT["Camara AT"]
    TEEAD --> V32AD["Valvula 3/2 NC (AD)"] --> AD["Camara AD"]
    V32AT --> EXH1["Desfogue a atmosfera"]
    V32AD --> EXH2["Desfogue a atmosfera"]

```

Notas:
- Valvulas **NC** por defecto. En reposo: **A -> R abierto**, **P cerrado**.
- Se usan 3/2 y cuando hace falta 2/2 se **tapa un puerto**.
- Check valves solo en el tanque (carga y descarga).
- Tanque metal: **0.3 L**, conectado en TEE de BOOST.
