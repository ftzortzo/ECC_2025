# Sioux Falls (TNTP) → SUMO → MATLAB (TraCI4Matlab)

This repository runs the **Sioux Falls** traffic assignment network in **SUMO** and allows you to control and step through the simulation from **MATLAB** using **TraCI4Matlab**.

**Features:**
* **Run out of the box:** Includes a prebuilt network and prebuilt routed demand.
* **Recreate everything:** Generate the network, TAZ, OD matrices, trips, and routes from scratch using the original **TNTP** files.

---

##  Repository Layout

**Repository Root:**
* `siouxfalls_gui.sumocfg` — SUMO configuration file (uses relative paths)

**Directories:**
* **`net/`**
    * `siouxfalls_utm_tls_fixed.net.xml` — Final network (UTM projected + traffic lights)
* **`demand/`**
    * `siouxfalls_10p_utm.rou.xml` — Default routed demand (10% scale)
    * `vtypes.add.xml` — Optional vehicle type file (only needed in some cases; see troubleshooting)
* **`tools/`**
    * `make_sumo_od_inputs.py` — Script to convert TNTP OD → `siouxfalls.taz.xml` + `siouxfalls.tazRel.xml`
    * `run_pipeline.ps1` — One-command PowerShell pipeline to regenerate net, TLS, demand, and routes from TNTP.
* **`tntp/`**
    * `SiouxFalls_node.tntp`
    * `SiouxFalls_net.tntp`
    * `SiouxFalls_trips.tntp`

---

##  Prerequisites

### 1. Install SUMO (Windows)

* Tested with **SUMO 1.26.0**
* **Important:** Set your environment variable `SUMO_HOME` to the SUMO installation directory (it should contain the `bin\` and `tools\` folders).

Verify your installation in PowerShell:
```powershell
& "$env:SUMO_HOME\bin\sumo-gui.exe" --version
```

### 2. Install MATLAB + TraCI4Matlab

* Tested with **MATLAB 2025b**
* Install TraCI4Matlab locally. For example, place it in `C:\Matlab_Work\`. You should have:
    * `C:\Matlab_Work\traci4matlab.jar`
    * `C:\Matlab_Work\+traci\...`

Run the following in MATLAB (once per session) to configure the paths:
```matlab
javaaddpath("C:\Matlab_Work\traci4matlab.jar");
addpath(genpath("C:\Matlab_Work"));
```

---

##  Quick Start: Run Out of the Box

### Step 1: Start SUMO-GUI as the TraCI Server
Open PowerShell, navigate to the repository root, and run:
```powershell
cd <repo>
& "$env:SUMO_HOME\bin\sumo-gui.exe" -c .\siouxfalls_gui.sumocfg --remote-port 8813 --num-clients 1 --start --delay 50
```
*In the SUMO-GUI:* Go to **View → Zoom to Extent**. Leave the GUI open; MATLAB will control the time advancement.

### Step 2: Connect MATLAB and Step the Simulation
Run the following script in MATLAB:
```matlab
try, traci.close(); catch, end
clear global connections message

% Ensure paths are set
javaaddpath("C:\Matlab_Work\traci4matlab.jar");
addpath(genpath("C:\Matlab_Work"));

% Initialize connection
[traciV, sumoV] = traci.init(8813, 10, '127.0.0.1', 'default');
disp(traciV); disp(sumoV);

% Get Traffic Light IDs
tls_ids = traci.trafficlights.getIDList();
disp(tls_ids);  % Example output: {'10'}

% Run simulation loop
nSteps = 5000;
for k = 1:nSteps
    traci.simulationStep();
    
    if mod(k, 200) == 0
        t = traci.simulation.getTime();
        dep = traci.simulation.getDepartedNumber();
        arr = traci.simulation.getArrivedNumber();
        active = numel(traci.vehicle.getIDList());
        fprintf("t=%.1f departed=%d arrived=%d active=%d\n", t, dep, arr, active);
    end
end

% Clean up
traci.close();
clear global connections message
```

---

##  Regenerate Demand & Routes (Reproducible Pipeline)

You can rebuild the network, routes, and intermediate artifacts (`work/`, `siouxfalls.taz.xml`, `siouxfalls.tazRel.xml`, `demand/siouxfalls.trips.xml`) directly from the source TNTP files.

### Step 0: Confirm TNTP Inputs Exist
Ensure the following files are present in the `tntp/` directory:
* `SiouxFalls_node.tntp`
* `SiouxFalls_net.tntp`
* `SiouxFalls_trips.tntp`

### Step 1: Run the Pipeline
From the repository root in PowerShell:
```powershell
cd <repo>
.\tools\run_pipeline.ps1 -Scale 0.1 -Begin 0 -End 3600
```
* `-Scale 0.1`: Generates 10% demand (ideal for debugging). Increase to `0.2`, `0.5`, or `1.0` as needed.
* `-Begin` / `-End`: Simulation interval in seconds.

### Step 2: Update Configuration
If you generate demand at a different scale, update the route file reference in `siouxfalls_gui.sumocfg`:
```xml
<route-files value="demand/siouxfalls_20p_utm.rou.xml"/>
```

> ###  Important Note: Vehicle Type `car`
> Sometimes `duarouter` outputs a route file that already contains `<vType id="car" ... />`. If you also load `demand/vtypes.add.xml` via `<additional-files>`, SUMO will throw a duplicate ID error:
> *"Another vehicle type (or distribution) with the id 'car' exists."*
> 
> **How to check:**
> ```powershell
> Select-String -Path .\demand\siouxfalls_10p_utm.rou.xml -Pattern "<vType id=""car""" -List
> ```
> * **If it prints a match:** Do **NOT** enable `<additional-files>` in `siouxfalls_gui.sumocfg`.
> * **If it prints nothing:** You must enable it in your config:
>     ```xml
>     <additional-files value="demand/vtypes.add.xml"/>
>     ```
>     *(Ensure `vtypes.add.xml` contains: `<additional><vType id="car" vClass="passenger"/></additional>`)*

---

##  How the Demand Pipeline Works

The TNTP OD data (`SiouxFalls_trips.tntp`) represents zone-to-zone flow, not individual vehicles. Here is how it gets converted:

1.  **`make_sumo_od_inputs.py`**: Generates `siouxfalls.taz.xml` (maps zones to source/sink edges) and `siouxfalls.tazRel.xml` (defines OD relations for a time interval).
2.  **`od2trips`**: Converts OD relations into individual trips with specific departure times.
3.  **`duarouter`**: Converts the generic trips into routed vehicles (`*.rou.xml`) that SUMO can actively simulate.

---

##  Troubleshooting

**1. SUMO error: "Invalid network, no network version declared"**
Your `<net-file>` is pointing to a configuration file (like a netedit/netconvert config) instead of a compiled SUMO network. A valid network file must start with `<net version="...">`. 
* **Fix:** Ensure your config points to `net/siouxfalls_utm_tls_fixed.net.xml`.

**2. SUMO pauses after "Starting server on port 8813"**
Because you passed `--num-clients 1`, SUMO is intentionally waiting for a TraCI client to connect.
* **Fix:** Start MATLAB and run `traci.init(...)`.

**3. MATLAB TraCI error: "Software caused connection abort"**
SUMO rejected an input (e.g., invalid network, route mismatch, or duplicate vType) and crashed/closed the connection.
* **Fix:** Run SUMO directly from the command line without TraCI to see the underlying error:
    ```powershell
    sumo -c .\siouxfalls_gui.sumocfg --no-step-log --verbose
    ```

**4. Nothing is visible in the GUI**
* **Fix:** In SUMO, go to **View → Zoom to Extent**. You may also need to increase the vehicle scale in the view settings. Ensure MATLAB is actually calling `traci.simulationStep()` to advance the frames.

---

##  References
* [SUMO Documentation](https://sumo.dlr.de/docs/index.html)
* [TransportationNetworks Sioux Falls Dataset](https://github.com/bstabler/TransportationNetworks)