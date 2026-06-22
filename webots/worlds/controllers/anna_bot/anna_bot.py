from pathlib import Path
import runpy


REPO_ROOT = Path(__file__).resolve().parents[4]
REAL_CONTROLLER = REPO_ROOT / 'webots' / 'controllers' / 'anna_bot' / 'anna_bot.py'

runpy.run_path(str(REAL_CONTROLLER), run_name='__main__')
