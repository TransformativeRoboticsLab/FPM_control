# FPM_control
For rotary control of FPM systems

## Python setup (Linux)

### 1) Create and activate a virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

If `.venv` already exists, only run:

```bash
source .venv/bin/activate
```

### 2) Install dependencies

```bash
pip install -r requirements.txt
```

### 3) Run scripts

```bash
python main.py
```

Other entry points:

```bash
python FPM_probe_surface.py
python FPM_search_space_and_track.py
```
