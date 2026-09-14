---
title: OD Route Output
---

# Instantiating within the Simulation

The OD route output is activated by setting the simulation option **--od-route-output** {{DT_FILE}} on the command line or in a *.sumocfg* file.
It counts, for every origin-destination pair, how many vehicles used each distinct route (edge sequence) and is intended for analyzing the route choice (route flow distribution) between traffic assignment zones (TAZ) or between edges.

The following sub-options are available:

| Option | Description |
| ------ | ----------- |
| **--od-route-output.period** {{DT_TIME}} | Aggregate over intervals of the given length (by vehicle arrival time) instead of the whole simulation; *default:* **-1** |
| **--od-route-output.write-unfinished** {{DT_BOOL}} | Also count the routes of vehicles which have departed but not yet arrived at simulation end; *default:* **false** |
| **--od-route-output.edges** {{DT_BOOL}} | Aggregate by departure and arrival edge instead of TAZ; *default:* **false** |

## Origin and destination

The origin and destination of a vehicle are derived from the first and the last edge of the route it actually drove (i.e. after any rerouting):

- If TAZ definitions are loaded (a [TAZ file](../../Demand/Importing_O/D_Matrices.md#describing_the_taz) given as {{AdditionalFiles}}, or option **--junction-taz**), the departure edge is mapped to the TAZ that lists it as a source edge and the arrival edge to the TAZ that lists it as a sink edge. The output element is then `tazRelation`.
  The vehicle attributes `fromTaz` and `toTaz` are **not** used; only the TAZ definitions decide the aggregation. Edges which are not part of any TAZ are reported with their own edge id. If an edge belongs to several TAZ, the alphabetically first TAZ id is used.
- If no TAZ are loaded or option **--od-route-output.edges** is set, the departure and arrival edge ids are used directly and the output element is `edgeRelation`.

Vehicles are counted once when they arrive. Routes with the same sequence of edges are merged, regardless of their route ids.

As with all outputs, setting the file extension to *.csv* or *.parquet* writes the same data in [tabular form](../../TabularOutputs.md).
Since counting happens only on vehicle arrival, the output has no measurable impact on the simulation speed.

# Generated Output

The generated XML file looks like this:

```xml
<odRoutes>
    <interval begin="0.00" end="300.00">
        <tazRelation from="1" to="2" count="10">
            <route id="1_2_0" edges="beg middle end rend" count="5" probability="0.50" length="2500.00" traveltime="134.20"/>
            <route id="1_2_1" edges="beg beg2left left left2end end rend" count="3" probability="0.30" length="3512.36" traveltime="180.67"/>
            <route id="1_2_2" edges="beg2left left left2end end" count="2" probability="0.20" length="2512.36" traveltime="132.50"/>
        </tazRelation>
        ...
    </interval>
    ...
</odRoutes>
```

With **--od-route-output.edges** (or without loaded TAZ) the `tazRelation` elements are replaced by `edgeRelation` elements with the same attributes.

## interval

| Attribute Name | Value Type | Description                                                                  |
| -------------- | ---------- | ---------------------------------------------------------------------------- |
| **begin**      | (time)     | Begin of the aggregation interval                                            |
| **end**        | (time)     | End of the aggregation interval (the simulation end for the last interval)   |

## tazRelation / edgeRelation

| Attribute Name | Value Type | Description                                                                  |
| -------------- | ---------- | ---------------------------------------------------------------------------- |
| **from**       | id         | Origin TAZ (or departure edge)                                               |
| **to**         | id         | Destination TAZ (or arrival edge)                                            |
| **count**      | #          | Number of vehicles that arrived between this origin and destination          |

## route

The routes of one relation are sorted by descending count.

| Attribute Name  | Value Type | Description                                                                  |
| --------------- | ---------- | ---------------------------------------------------------------------------- |
| **id**          | id         | Generated id of the form `<from>_<to>_<index>`                               |
| **edges**       | id list    | The sequence of edges that make up the route                                 |
| **count**       | #          | Number of vehicles that used this route                                      |
| **probability** | [0,1]      | Share of this route among all vehicles of the relation                       |
| **length**      | m          | Length of the route (sum of edge lengths, including internal edges if used)  |
| **traveltime**  | s          | Mean travel time (departure to arrival) of the vehicles that completed this route; -1 if none did (only possible with **--od-route-output.write-unfinished**) |
