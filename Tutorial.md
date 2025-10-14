# Step-by-Step Guide to Connecting MATLAB/Simulink and Unreal Engine with Integrated VR 


##  Step 1 — Install MATLAB and Required Toolboxes

1. Download MATLAB from [MathWorks Downloads](https://www.mathworks.com/downloads/).
2. During installation, include the following toolboxes:
   - **Simulink**
   - **Automated Driving Toolbox**
   - **Vehicle Dynamics Blockset**
   - **Unreal Engine Interface for Vehicle Dynamics Blockset**
   - *(Optional)* **3D Animation Toolbox**
3. After installation, activate your MATLAB license.
4. Verify your installation by running:
   ```matlab
   ver


---

###  Step 2 — Install Epic Games Launcher and Unreal Engine

1. Download and install the **Epic Games Launcher**:  
   [https://www.epicgames.com/store/en-US/download](https://www.epicgames.com/store/en-US/download)
2. Sign in or create an **Epic Games account**.
3. In the launcher:
   - Go to **Unreal Engine → Library**
   - Click **Install Engine**
   - Select **Unreal Engine 5.x** (recommended: **5.3 or newer**)
4. After installation, **launch Unreal Engine once** to complete setup.

>  MATLAB officially supports Unreal Engine **4.26** by default.  
> To connect to Unreal 5.x, you will later generate and install the MATLAB plugin manually.


##  Step 3 — Install RoadRunner (Optional, for Custom Scene Creation)

1. Download **MathWorks RoadRunner** from:  
   [https://www.mathworks.com/downloads/](https://www.mathworks.com/downloads/)
2. Install and activate it using your MathWorks license.
3. When you first launch RoadRunner:
   - Choose or create a workspace folder (e.g., `~/Documents/RoadRunner`)
4. Inside RoadRunner, you can:
   - Build roads, intersections, and city environments.
   - Export them as `.rrproject` or `.umap` files for Unreal Engine.


## 🔌 Step 4 — Install and Link the MATLAB–Unreal Plugin

1. Open MATLAB.
2. Run the following command to set up the Unreal Engine interface:
   ```matlab
   vehdynlib.unrealengine.setup



---

###  Step 5 — Create or Open an Unreal Project

1. Launch **Unreal Engine**.
2. From the project wizard:
   - Choose **Games → Blank Project**
   - Select **Blueprint**
   - Disable **Starter Content**
   - Click **Create Project**
3. Once the project opens:
   - Go to **Edit → Plugins → Installed → Vehicle Dynamics Blockset Interface**
   - Enable the plugin
   - Restart Unreal Engine when prompted

