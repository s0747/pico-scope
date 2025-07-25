DIAGRAM.md

https://mermaid.js.org/syntax/flowchart.html

```mermaid
---
title: "Grades"
---
radar-beta
  axis m["Math"], s["Science"], e["English"]
  axis h["History"], g["Geography"], a["Art"]
  curve a["Alice"]{85, 90, 80, 70, 75, 90}
  curve b["Bob"]{70, 75, 85, 80, 90, 85}

  max 100
  min 0
```

---

```mermaid
---
config:
  pie:
    textPosition: 0.5
  themeVariables:
    pieOuterStrokeWidth: "5px"
---
pie showData
    title Key elements in Product X
    "Calcium" : 42.96
    "Potassium" : 50.05
    "Magnesium" : 10.01
    "Iron" :  5
```

---


```mermaid
xychart-beta
    title "Sales Revenue"
    x-axis [jan, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec]
    y-axis "Revenue (in $)" 4000 --> 11000
    bar [5000, 6000, 7500, 8200, 9500, 10500, 11000, 10200, 9200, 8500, 7000, 6000]
    line [5000, 6000, 7500, 8200, 9500, 10500, 11000, 10200, 9200, 8500, 7000, 6000]

```
```mermaid
xychart-beta
    title "Sales Revenue"
    x-axis [jan, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec]
    y-axis "Revenue (in $)" 4000 --> 11000
    bar [5000, 6000, 7500, 8200, 9500, 10500, 11000, 10200, 9200, 8500, 7000, 6000]

```

Test  table:
| Functions                | Code         | Access | Data                     | Implemented |
|--------------------------|--------------|--------|--------------------------|-------------|
| manufacturer_access*     | 0x00         | r/w    | word                     | ❌          |
| remaining_capacity_alarm | 0x01         | r/w    | mAh or 10mWh             | ✅          |


Test diagram:

## Design Principles
### Generic Design Diagram
```mermaid
    classDiagram
        Application Level Task --> ChargerDriver: non-std calls (ex. MFG)
        Application Level Task --> SmartBatteryDriver: non-std calls (ex. MFG)
        Application Level Task: -> charging FSMs
        Application Level Task: -> monitoring state of charge
        Application Level Task: -> calls std and non-std fn's
        Application Level Task: +do_startup()
        Application Level Task: +enable_autocharging()
        Application Level Task: +get_cell_voltages()
        Application Level Task: +...()
        Application Level Task --> embedded-batteries-charger: std calls
        Application Level Task --> embedded-batteries-smart-battery: std calls
        embedded-batteries-smart-battery <|-- SmartBatteryDriver: implements
        embedded-batteries-charger <|-- ChargerDriver: implements
        embedded-batteries-charger: -> Platform/HW agnostic charger fn's
        embedded-batteries-charger: -> Conforms to SBS spec
        embedded-batteries-charger: +charging_current()
        embedded-batteries-charger: +charging_voltage()
        embedded-batteries-smart-battery: -> Platform/HW agnostic fuel gauge fn's
        embedded-batteries-smart-battery: -> Conforms to SBS spec
        embedded-batteries-smart-battery: +voltage()
        embedded-batteries-smart-battery: +current()
        embedded-batteries-smart-battery: +battery_status()
        embedded-batteries-smart-battery: +temperature()
        embedded-batteries-smart-battery: +cycle_count()
        embedded-batteries-smart-battery: +...()
        <<interface>> embedded-batteries-charger
        <<interface>> embedded-batteries-smart-battery
        ChargerDriver --> platform-specific-hal
        ChargerDriver: -> Specific to charging chip
        ChargerDriver: -> Conforms to chip datasheet
        ChargerDriver: -> has non-std MFG specific fn's
        ChargerDriver: +[std] charging_current()
        ChargerDriver: +[std] charging_voltage()
        ChargerDriver: +[non-std] auto_charge()
        ChargerDriver: +[non-std] ...()
        SmartBatteryDriver --> platform-specific-hal
        SmartBatteryDriver: -> Specific to charging chip
        SmartBatteryDriver: -> Conforms to chip datasheet
        SmartBatteryDriver: -> has non-std MFG specific fn's
        SmartBatteryDriver: +[std] voltage()
        SmartBatteryDriver: +[std] current()
        SmartBatteryDriver: +[std] battery_status()
        SmartBatteryDriver: +[std] ...()
        SmartBatteryDriver: +[non-std] unseal()
        SmartBatteryDriver: +[non-std] ...()
        class platform-specific-hal["platform-specific-hal comm system"]
        platform-specific-hal: -> communication system agnostic
        platform-specific-hal: -> can be uC, kernel driver, etc.
        platform-specific-hal: +read_word()
        platform-specific-hal: +write_word()
        platform-specific-hal: +...()
```
