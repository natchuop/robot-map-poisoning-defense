# Equations and Simulation Measurements for Trust-Based Map Poisoning Defense

This document gives implementation-level context for the equations, state variables, update rules, and measurements used in the final multi-robot map-poisoning defense experiment.

The main project claim is:

```text
Can decentralized trust-weighted map fusion reduce the effects of map-poisoning attacks on multi-robot navigation and final map accuracy?
```

The simulation compares three map-fusion models:

```text
1. log_odds
2. mate_log_odds
3. mate_claim_verification
```

The first attack type is fake obstacle insertion, where a compromised robot reports a free cell as occupied. Fake clearing, where a compromised robot reports a real occupied cell as free, is important but should be treated as future or secondary work.

## Common Rule: Direct Local Observation Precedence

All three fusion modes use the robot's own current LiDAR observation as the highest-priority evidence for cells that are currently visible.

If the current observation says a cell is free, the robot should clear or strongly downgrade any remote occupied claim for that cell. If the current observation says a cell is occupied, the robot should preserve the occupied state even if remote reports say free.

Cells with `current_observation_map[cell] = -1` are unknown in the latest scan and must not affect the merge.

### LiDAR Range Quality

Current-observation evidence is now range-weighted instead of binary. Near returns should act like strong evidence, while distant returns should contribute less confidence to both cell coloring and occupancy updates.

The shared implementation uses a smoothstep falloff:

```text
q(r) = 1.0                               for r <= r_near
q(r) = q_min                             for r >= r_far
q(r) = q_min + (1 - q_min) * (1 - s(t))  otherwise

t = (r - r_near) / (r_far - r_near)
s(t) = t^2 * (3 - 2t)
```

Recommended defaults:

```text
r_near = 1.25 m
r_far = 4.0 m
q_min = 0.15
max_free_clear_range = 3.0 m
```

In code, the observation strength is scaled by range quality:

```text
hit_delta = hit_score_increment * q(r)
free_delta = free_score_decrement * q(r)
```

Free-space clearing is capped so that far beams do not erase too much of the map:

```text
if free_cell_distance > max_free_clear_range:
    stop clearing that ray
```

The current-observation map should also encode weaker distant evidence with midrange values instead of only `0` and `100`, so the visualized cell color reflects confidence as well as occupancy state.

This is not a trust-defense rule by itself. In Method 1, remote claims still use full trust when the cell is outside the robot's current observation. The same precedence rule also applies in Method 2 and Method 3, which differ only in how remote claims are weighted and later verified.

Method 1 is implemented as the full-trust log-odds baseline with current-observation override. In code, fake obstacle reports are carried as `MapUpdate` claims and fused into the receiver's shared map using the log-odds path, then the viewing robot's own current LiDAR observation can clear contradicted occupied claims in currently visible cells.
That current observation is now range-weighted, so a cell seen at the end of the LiDAR range contributes less confidence than a nearby cell.

The first required maps are:

```text
1. office
2. single_hallway
3. two_path
```

A small maze can be added later as an optional stress test.

---

## 1. Common Notation

| Symbol / variable | Meaning |
|---|---|
| `i` | Receiving, observing, or verifying robot ID |
| `j` | Reporting robot ID |
| `c` | Occupancy-grid cell being updated |
| `t` | Current time or update step |
| `cell_x`, `cell_y` | Integer grid coordinates for cell `c` |
| `claim_id` | Unique ID for a shared map claim |
| `reported_state` | State reported by another robot: `OCCUPIED` or `FREE` |
| `ground_truth_state` | True state of cell in simulation: `OCCUPIED` or `FREE` |
| `is_attack_report` | Simulation-only flag used for evaluation, never defense logic |
| `P_occ(c)` | Occupancy probability or occupancy-like score of cell `c` |
| `L_c` | Log-odds value for cell `c` |
| `O_c` | Occupied evidence stored for cell `c` |
| `F_c` | Free or clear evidence stored for cell `c` |
| `epsilon` | Small constant to avoid division by zero |
| `alpha_ij`, `beta_ij` | MATE-style trust parameters maintained by observer `i` for reporter `j` |
| `alpha0`, `beta0` | Neutral trust prior parameters, usually `1.0` and `1.0` |
| `tau_ij` | Random trust variable for reporter `j` as estimated by observer `i` |
| `T_ij` | MATE trust mean, `alpha_ij / (alpha_ij + beta_ij)` |
| `nu_ij` | Trust precision, `alpha_ij + beta_ij` |
| `n_eff_ij` | Effective verification evidence count after removing the neutral prior |
| `rho_ijc` | Trust pseudomeasurement for reporter `j` about claim/cell `c` |
| `v_ijc` | Pseudomeasurement value in `[0, 1]`: `1` supports trust, `0` supports distrust |
| `c_ijc` | Pseudomeasurement confidence in `[0, 1]` |
| `C_ij` | Confidence in the MATE trust estimate |
| `Q_ijc` | Quality of robot `j`'s report about cell `c` |
| `R_ijc` | Verification confidence for robot `j`'s claim about cell `c` |
| `lambda_ij` | Caution factor for reporter `j` from observer `i`'s perspective |
| `ramp_ij` | Update ramp, `1 - lambda_ij` |
| `w_ijc` | Final Method 3 influence weight for robot `j`'s report about cell `c` |

Recommended shared constants:

```text
epsilon = 1e-6
l_occ = 0.85
l_free = -0.40
L_min = -5.0
L_max = 5.0

mate_alpha0 = 1.0
mate_beta0 = 1.0
mate_prior_pull_omega = 0.001 to 0.01
mate_negative_bias = 2.0 or 3.0
mate_negative_threshold = 0.5
mate_min_psm_confidence = 0.2

trust_confidence_k = 10
lambda_initial_caution = 0.9
lambda_decay_gamma = 0.1
recent_window_size = 20
recent_window_seconds = 60.0
sensor_distance_sigma = 3.0 meters
report_age_tau = 10.0 seconds
evidence_alpha = 0.99 or 1.0
occupied_evidence_cap = 10.0
free_evidence_cap = 10.0
theta_unknown_evidence = 0.5
theta_occupied = 0.7
theta_free = 0.3
theta_dispute = 0.5
theta_quarantine_trust = 0.25
theta_quarantine_confidence = 0.70
```

The three final fusion modes are:

```text
log_odds
mate_log_odds
mate_claim_verification
```

The main design separation is:

```text
Method 1: no trust, all robots effectively have 100% trust.
Method 2: MATE-style robot trust only, fused with ordinary trust-weighted log-odds.
Method 3: MATE-style trust plus claim-level quality, verification, evidence layers, suspicious/disputed states, and quarantine.
```

## 2. Model 1: Standard Bayesian Occupancy Grid / Log-Odds

### 2.1 Purpose

This is the industry-standard robotics-style baseline. It does not use robot trust. Every incoming map update is treated equally, regardless of which robot sent it.

In practice, this means the receiving robot behaves as if every other robot has full or equal trust.

Direct current LiDAR still has precedence for currently visible cells. Method 1 only accepts remote claims at full trust when the robot does not have contradictory current sensor evidence for that cell.

This model answers:

```text
What happens when the navigation system accepts shared occupancy updates without a trust defense?
```

### 2.2 Cell State

Each cell stores one log-odds value:

```text
L_c(t) = log(P_c(t) / (1 - P_c(t)))
```

where:

```text
P_c(t) = probability that cell c is occupied at time t
L_c(t) = log-odds representation of P_c(t)
```

A neutral unknown cell can start at:

```text
P_c(0) = 0.5
L_c(0) = 0.0
```

### 2.3 Update Rule

If a map update reports that cell `c` is occupied:

```text
L_c(t+1) = L_c(t) + l_occ
```

If a map update reports that cell `c` is free:

```text
L_c(t+1) = L_c(t) + l_free
```

Recommended starting values:

```text
l_occ = 0.85
l_free = -0.40
```

The final occupancy probability is recovered using:

```text
P_occ(c) = 1 / (1 + exp(-L_c))
```

### 2.4 Optional Clamping

To prevent the log-odds value from growing too large, clamp it:

```text
L_c = max(L_min, min(L_c, L_max))
```

Suggested values:

```text
L_min = -5.0
L_max = 5.0
```

### 2.5 Classification

A simple classification rule can be:

```text
if P_occ(c) > 0.7:
    state = OCCUPIED
elif P_occ(c) < 0.3:
    state = CLEAR
else:
    state = UNKNOWN
```

This model does not naturally support `SUSPICIOUS`, `DISPUTED`, or `QUARANTINED` states.

### 2.6 Pseudocode

```text
for each incoming map update about cell c:
    if reported_state == OCCUPIED:
        L[c] = L[c] + l_occ
    else if reported_state == FREE:
        L[c] = L[c] + l_free

    L[c] = clamp(L[c], L_min, L_max)
    P_occ[c] = 1 / (1 + exp(-L[c]))

    if P_occ[c] > theta_occupied:
        state[c] = OCCUPIED
    else if P_occ[c] < theta_free:
        state[c] = CLEAR
    else:
        state[c] = UNKNOWN
```

### 2.7 Expected Behavior Under Fake Obstacles

If a malicious robot reports a fake obstacle, the update is accepted like any other occupied report. In the single-hallway map, this may block the only path. In the two-path map, this may force the robot to take the longer path. In the office map, this may cause unnecessary rerouting around hallways or doorways.

---

## 3. Model 2: MATE-Weighted Log-Odds Baseline

### 3.1 Purpose

This model replaces the older simple beta-reputation baseline with a stronger MATE-style robot trust baseline. It still keeps the standard occupancy-grid/log-odds map update, but it weights incoming updates using a MATE-style Bayesian trust estimate.

This model answers:

```text
If robots use MATE-style trust estimation, but still use ordinary trust-weighted log-odds fusion, how much does that improve map-poisoning resistance over full trust?
```

This model intentionally does **not** include the Method 3 additions. It should not use claim-level map-fusion weights such as `C_ij`, `Q_ijc`, `R_ijc`, caution ramping, occupied/free evidence layers, suspicious/disputed states, claim-level evidence removal, or quarantine.

Method 2 includes only:

```text
MATE-style trust PDFs
optional MATE-style trust propagation
trust pseudomeasurements from verification receipts
simple trust-weighted log-odds fusion
```

### 3.2 MATE-Style Robot Trust PDF

Each receiving or observing robot `i` keeps a trust distribution for every reporting robot `j`:

```text
tau_ij(t) ~ Beta(alpha_ij(t), beta_ij(t))
```

Use a neutral prior:

```text
alpha_ij(0) = alpha0 = 1.0
beta_ij(0) = beta0 = 1.0
```

The MATE trust mean is:

```text
T_ij(t) = alpha_ij(t) / (alpha_ij(t) + beta_ij(t))
```

The trust precision is:

```text
nu_ij(t) = alpha_ij(t) + beta_ij(t)
```

Interpretation:

```text
T_ij close to 1.0 -> reporter j appears trustworthy to observer i
T_ij close to 0.0 -> reporter j appears untrustworthy to observer i
T_ij close to 0.5 -> unknown, neutral, or uncertain trust
nu_ij high -> the estimate is supported by more evidence
nu_ij low -> the estimate is weak or new
```

### 3.3 Optional MATE-Style Trust Propagation

This project will use optional MATE-style trust propagation. Before applying new verification evidence, the trust PDF can be pulled slightly back toward the neutral prior:

```text
alpha_ij_minus(t) = (1 - omega) * alpha_ij(t-1) + omega * alpha0
beta_ij_minus(t)  = (1 - omega) * beta_ij(t-1)  + omega * beta0
```

where:

```text
omega = mate_prior_pull_omega
```

Recommended starting range:

```text
omega = 0.001 to 0.01
```

Interpretation:

```text
If there is no recent evidence, trust slowly moves back toward uncertainty.
This helps with late attackers because old good behavior should not permanently dominate the trust estimate.
```

If propagation is temporarily disabled for debugging:

```text
omega = 0.0
alpha_ij_minus = alpha_ij(t-1)
beta_ij_minus = beta_ij(t-1)
```

### 3.4 Trust Pseudomeasurements from Verification Receipts

There is no physical trust sensor. Instead, verification results are converted into a trust pseudomeasurement:

```text
rho_ijc(t) = (v_ijc(t), c_ijc(t))
```

where:

```text
v_ijc(t) = trust evidence value in [0, 1]
c_ijc(t) = confidence in that evidence in [0, 1]
```

For fake obstacle insertion, where the original claim is `OCCUPIED`:

```text
if later LiDAR confirms an obstacle at cell c:
    verification_result = CONFIRMED
    v_ijc = 1.0
    c_ijc = verification quality

elif later LiDAR passes through cell c as free space:
    verification_result = CONTRADICTED
    v_ijc = 0.0
    c_ijc = verification quality

else:
    verification_result = UNCERTAIN
    no trust update
```

For Method 2, keep the verification quality simple so it does not overlap too much with Method 3:

```text
direct reliable LiDAR verification: c_ijc = 1.0
weaker but usable verification:     c_ijc = 0.5
uncertain observation:              no update
```

Method 2 can use verification to update robot trust, but it should not use `Q_ijc`, `R_ijc`, or caution ramping as map-fusion weights.

### 3.5 MATE Trust Update

Without negative bias:

```text
alpha_ij(t) = alpha_ij_minus(t) + c_ijc(t) * v_ijc(t)
beta_ij(t)  = beta_ij_minus(t)  + c_ijc(t) * (1 - v_ijc(t))
```

For security, negative evidence can be weighted more strongly than positive evidence:

```text
omega_neg = mate_negative_bias if v_ijc < mate_negative_threshold else 1.0
```

Then:

```text
alpha_ij(t) = alpha_ij_minus(t) + c_ijc(t) * v_ijc(t)
beta_ij(t)  = beta_ij_minus(t)  + omega_neg * c_ijc(t) * (1 - v_ijc(t))
```

Recommended starting values:

```text
mate_negative_bias = 2.0 or 3.0
mate_negative_threshold = 0.5
```

Interpretation:

```text
Confirmed claims slowly increase trust.
Contradicted claims reduce trust faster.
Uncertain claims do not affect trust.
```

### 3.6 MATE-Weighted Log-Odds Update

After trust is propagated and updated, compute:

```text
T_ij(t) = alpha_ij(t) / (alpha_ij(t) + beta_ij(t))
```

If robot `j` reports cell `c` as occupied:

```text
L_c(t+1) = L_c(t) + T_ij(t) * l_occ
```

If robot `j` reports cell `c` as free:

```text
L_c(t+1) = L_c(t) + T_ij(t) * l_free
```

Then:

```text
P_occ(c) = 1 / (1 + exp(-L_c))
```

This model uses only the MATE trust mean as the map update multiplier. It does not create suspicious or disputed cell states.

### 3.7 Classification

Use the same classification as Method 1:

```text
if P_occ(c) > theta_occupied:
    state = OCCUPIED
elif P_occ(c) < theta_free:
    state = CLEAR
else:
    state = UNKNOWN
```

### 3.8 Pseudocode

```text
for each incoming map update from robot j about cell c:

    # Optional MATE-style propagation
    alpha_minus[j] = (1 - omega) * alpha[j] + omega * alpha0
    beta_minus[j]  = (1 - omega) * beta[j]  + omega * beta0

    # Use current trust as the map-fusion weight
    T = alpha_minus[j] / (alpha_minus[j] + beta_minus[j])

    if reported_state == OCCUPIED:
        L[c] = L[c] + T * l_occ
    else if reported_state == FREE:
        L[c] = L[c] + T * l_free

    L[c] = clamp(L[c], L_min, L_max)
    P_occ[c] = 1 / (1 + exp(-L[c]))
    classify cell using P_occ thresholds

for each verification receipt about a previous claim from robot j:

    convert receipt to rho = (v, c)

    if receipt is CONFIRMED or CONTRADICTED:
        omega_neg = mate_negative_bias if v < mate_negative_threshold else 1.0
        alpha[j] = alpha_minus[j] + c * v
        beta[j]  = beta_minus[j]  + omega_neg * c * (1 - v)
    else:
        no trust update
```

### 3.9 Expected Behavior Under Fake Obstacles

If the attacker has already been contradicted, fake obstacle reports have reduced influence. If the attacker is new, the neutral MATE trust mean starts at `0.5`, so the fake obstacle may still have moderate influence. If the attacker behaved honestly before attacking, optional trust propagation and negative evidence help, but the method can still leave poisoned evidence inside the log-odds map.

This model is useful because it shows whether MATE-style robot trust alone is enough, or whether the proposed claim-level verification system is needed.

## 4. Model 3: Proposed MATE-Based Claim Verification Defense

### 4.1 Purpose

This is the proposed defense. It extends the MATE-style trust baseline from Method 2 with map-poisoning-specific claim-level reasoning.

Method 3 answers:

```text
Does decentralized claim-level verification improve navigation and final map accuracy beyond MATE-style robot trust alone?
```

The key distinction is:

```text
Method 2 asks: how trustworthy is this robot?
Method 3 asks: how much should this specific robot's specific claim about this specific cell affect navigation right now?
```

Method 3 uses MATE-style robot trust, trust confidence from trust precision, optional lifetime/recent trust, caution ramping, report quality, verification confidence, occupied/free evidence layers, suspicious/disputed states, claim-level evidence removal, and optional quarantine.

### 4.2 MATE Trust Foundation

Each observer `i` maintains MATE-style trust PDFs for each reporter `j`.

For the proposed method, use lifetime and recent trust so the system can handle late attackers:

```text
tau_life_ij(t)   ~ Beta(alpha_life_ij(t), beta_life_ij(t))
tau_recent_ij(t) ~ Beta(alpha_recent_ij(t), beta_recent_ij(t))
```

Lifetime trust:

```text
T_life_ij(t) = alpha_life_ij(t) / (alpha_life_ij(t) + beta_life_ij(t))
```

Recent trust:

```text
T_recent_ij(t) = alpha_recent_ij(t) / (alpha_recent_ij(t) + beta_recent_ij(t))
```

Combined trust:

```text
T_ij(t) = min(T_life_ij(t), T_recent_ij(t))
```

Using the minimum makes the system cautious when a historically good robot starts behaving badly.

### 4.3 Optional MATE-Style Trust Propagation

Apply the same optional propagation idea from Method 2 to lifetime and recent trust before each trust update:

```text
alpha_life_minus = (1 - omega) * alpha_life + omega * alpha0
beta_life_minus  = (1 - omega) * beta_life  + omega * beta0
```

```text
alpha_recent_minus = (1 - omega) * alpha_recent + omega * alpha0
beta_recent_minus  = (1 - omega) * beta_recent  + omega * beta0
```

Recommended:

```text
omega = mate_prior_pull_omega = 0.001 to 0.01
```

This keeps trust dynamic and prevents old behavior from dominating forever.

### 4.4 Trust Pseudomeasurement Update

Verification receipts are converted into MATE-style pseudomeasurements:

```text
rho_ijc(t) = (v_ijc(t), c_ijc(t))
```

For fake obstacle insertion:

```text
if original claim was OCCUPIED and verifier sees obstacle:
    v_ijc = 1.0
    c_ijc = verifier observation quality

elif original claim was OCCUPIED and verifier ray passes through as free:
    v_ijc = 0.0
    c_ijc = verifier observation quality

else:
    no trust update
```

Update lifetime and recent trust:

```text
omega_neg = mate_negative_bias if v_ijc < mate_negative_threshold else 1.0
```

```text
alpha_life_ij(t) = alpha_life_minus + c_ijc * v_ijc
beta_life_ij(t)  = beta_life_minus  + omega_neg * c_ijc * (1 - v_ijc)
```

The recent trust PDF is updated the same way, but only over the most recent `N` verified reports or most recent `M` seconds. Implementation can maintain a sliding buffer of recent pseudomeasurements and recompute recent `alpha` and `beta` from that buffer.

### 4.5 Trust Confidence from MATE Precision

Trust confidence measures how much evidence supports the trust estimate.

Use effective evidence count from the MATE trust precision:

```text
nu_life_ij(t) = alpha_life_ij(t) + beta_life_ij(t)
nu0 = alpha0 + beta0
n_eff_ij(t) = max(0, nu_life_ij(t) - nu0)
```

Then:

```text
C_ij(t) = n_eff_ij(t) / (n_eff_ij(t) + trust_confidence_k)
```

Recommended:

```text
trust_confidence_k = 10
```

Interpretation:

```text
T_ij says how trustworthy the reporter seems.
C_ij says how much evidence supports that trust estimate.
```

Trust confidence increases after both positive and negative verified evidence. A false report does not reduce confidence; it increases confidence that the robot may be unreliable.

### 4.6 Caution Ramp

New or weakly verified robots should not immediately have strong map influence.

```text
lambda_ij(t) = lambda_initial_caution * exp(-lambda_decay_gamma * n_eff_ij(t))
```

```text
ramp_ij(t) = 1 - lambda_ij(t)
```

Recommended:

```text
lambda_initial_caution = 0.9
lambda_decay_gamma = 0.1
```

At the beginning:

```text
n_eff = 0
lambda = 0.9
ramp = 0.1
```

After many verified reports:

```text
lambda approaches 0
ramp approaches 1
```

This term is Method 3 only. Method 2 should not use the caution ramp as a map-fusion multiplier.

### 4.7 Report Quality

Report quality measures whether this specific report about this specific cell is physically reliable.

```text
Q_ijc(t) = Q_range_ijc(t) * Q_age_ijc(t) * Q_visibility_ijc(t) * Q_duplicate_ijc(t)
```

#### 4.7.1 Range Quality

Reports about nearby cells are more reliable than reports about far cells.

```text
Q_range_ijc(t) = exp(-(d_jc(t)^2) / (2 * sigma_d^2))
```

where:

```text
d_jc(t) = distance from robot j's observation pose to the center of cell c
sigma_d = sensor distance scale
```

Recommended:

```text
sigma_d = 3.0 meters
```

#### 4.7.2 Age Quality

Recent reports are more reliable than stale reports.

```text
Q_age_ijc(t) = exp(-(t_current - t_report) / tau)
```

Recommended:

```text
tau = 10.0 seconds
```

If the report timestamp is in the future or invalid, clamp or reject the report.

#### 4.7.3 Visibility Quality

A robot should not strongly affect a cell it could not see.

Suggested values:

| Observation condition | `Q_visibility` |
|---|---:|
| Direct LiDAR hit on the cell | 1.0 |
| Cell lies along a free-space LiDAR ray | 0.8 |
| Near sensor field edge | 0.5 |
| Occluded by another obstacle | 0.0 |
| Outside sensor range | 0.0 |

#### 4.7.4 Duplicate Quality

Repeated copies of the same report should not repeatedly strengthen the map.

Hard rule:

```text
if same robot, same cell, same claim_id or same scan_id already processed:
    ignore duplicate
```

Soft rule:

```text
Q_duplicate_ijc = 1 / (1 + n_duplicate_ijc)
```

where:

```text
n_duplicate_ijc = number of recent repeated reports from robot j for cell c
```

### 4.8 Verification Confidence

Verification confidence measures whether the specific claim has been checked by physical evidence.

Suggested values:

| Verification condition | `R_ijc` |
|---|---:|
| Directly confirmed by this robot's LiDAR | 1.0 |
| Confirmed by another trusted robot | 0.8 |
| Consistent with existing verified map | 0.6 |
| New but unverified claim | 0.3 |
| Conflicts with trusted evidence | 0.0 to 0.2 |

For fake obstacle insertion, a new fake obstacle should usually start with low verification confidence:

```text
R_ijc = 0.3
```

After another robot observes the cell and sees that it is free:

```text
R_ijc = 0.0 to 0.2
verification_result = CONTRADICTED
```

### 4.9 Main Influence Equation

For every report from robot `j` about cell `c`, compute:

```text
w_ijc(t) = T_ij(t) * C_ij(t) * Q_ijc(t) * R_ijc(t) * ramp_ij(t)
```

Equivalent form:

```text
w_ijc(t) = T_ij(t) * C_ij(t) * Q_ijc(t) * R_ijc(t) * (1 - lambda_ij(t))
```

Clamp the final weight:

```text
w_ijc = max(0.0, min(w_ijc, 1.0))
```

If the robot is quarantined:

```text
w_ijc = 0.0
```

### 4.10 Occupied and Free Evidence Layers

Method 3 does not store only one log-odds value. It stores two evidence values per cell:

```text
O_c(t) = occupied evidence
F_c(t) = free evidence
```

If using stored-evidence aging:

```text
O_c(t) = evidence_alpha * O_c(t)
F_c(t) = evidence_alpha * F_c(t)
```

Recommended:

```text
evidence_alpha = 0.99 for slowly fading evidence
evidence_alpha = 1.0 for no stored-evidence decay
```

If robot `j` reports occupied:

```text
O_c(t+1) = min(occupied_evidence_cap, evidence_alpha * O_c(t) + w_ijc(t))
F_c(t+1) = evidence_alpha * F_c(t)
```

If robot `j` reports free:

```text
F_c(t+1) = min(free_evidence_cap, evidence_alpha * F_c(t) + w_ijc(t))
O_c(t+1) = evidence_alpha * O_c(t)
```

### 4.11 Occupancy Score

Compute occupancy-like probability:

```text
P_occ(c) = O_c / (O_c + F_c + epsilon)
```

Interpretation:

```text
P_occ close to 1.0 -> likely occupied
P_occ close to 0.0 -> likely clear
P_occ close to 0.5 -> uncertain or disputed
```

### 4.12 Dispute Score and Cell Classification

Definitions:

```text
total_evidence_c = O_c + F_c
```

```text
dispute_score_c = 2 * min(O_c, F_c) / (O_c + F_c + epsilon)
```

Recommended thresholds:

```text
theta_unknown_evidence = 0.5
theta_occupied = 0.7
theta_free = 0.3
theta_dispute = 0.5
```

Classification rule:

```text
if total_evidence_c < theta_unknown_evidence:
    state = UNKNOWN

else if O_c > theta_unknown_evidence and F_c > theta_unknown_evidence and dispute_score_c > theta_dispute:
    state = DISPUTED

else if P_occ(c) > theta_occupied:
    state = OCCUPIED

else if P_occ(c) < theta_free:
    state = CLEAR

else:
    state = SUSPICIOUS
```

Navigation interpretation:

| Cell state | Meaning | Navigation behavior |
|---|---|---|
| `UNKNOWN` | Not enough evidence | Use normal Nav2 unknown-space handling |
| `OCCUPIED` | Strong obstacle evidence | High/lethal cost |
| `CLEAR` | Strong free-space evidence | Normal traversal |
| `SUSPICIOUS` | Uncertain or weak evidence | Medium cost or verification target |
| `DISPUTED` | Strong occupied and free evidence both exist | Avoid if possible, request verification |

### 4.13 Claim-Level Evidence Removal

This is a key difference between Method 2 and Method 3. Method 2 can reduce a reporter's future trust, but old poisoned evidence may remain in the log-odds map. Method 3 stores each claim's contribution so a contradicted claim can be downgraded or removed.

For every accepted claim, store:

```text
claim_id
reporting_robot_id
cell_x
cell_y
reported_state
evidence_added = w_ijc at insertion time
active = true
```

If an occupied claim is contradicted:

```text
O_c = max(0, O_c - k_remove * evidence_added)
F_c = min(free_evidence_cap, F_c + k_verify * c_verify)
active = false
```

If a free claim is contradicted:

```text
F_c = max(0, F_c - k_remove * evidence_added)
O_c = min(occupied_evidence_cap, O_c + k_verify * c_verify)
active = false
```

Recommended starting values:

```text
k_remove = 1.0
k_verify = 1.0
```

### 4.14 Quarantine

A robot can be quarantined when it is both untrusted and confidently known to be unreliable.

```text
quarantine_ij = true if T_ij < theta_quarantine_trust and C_ij > theta_quarantine_confidence
```

Recommended:

```text
theta_quarantine_trust = 0.25
theta_quarantine_confidence = 0.70
```

If quarantined:

```text
w_ijc = 0 for all future reports from robot j
```

The reports should still be logged for analysis, but they should not affect the map or navigation.

### 4.15 Full Pseudocode

```text
for each incoming map update from robot j about cell c:

    if trust_manager.is_quarantined(i, j):
        log update with quarantined = true
        set w_ijc = 0
        return

    # MATE trust foundation
    propagate lifetime and recent trust PDFs using omega
    T_life = alpha_life[j] / (alpha_life[j] + beta_life[j])
    T_recent = alpha_recent[j] / (alpha_recent[j] + beta_recent[j])
    T = min(T_life, T_recent)

    # Trust confidence and cautious onboarding
    n_eff = max(0, alpha_life[j] + beta_life[j] - alpha0 - beta0)
    C = n_eff / (n_eff + trust_confidence_k)
    lambda_value = lambda_initial_caution * exp(-lambda_decay_gamma * n_eff)
    ramp = 1 - lambda_value

    # Claim-specific map influence terms
    Q_range = compute_range_quality(update.robot_pose_when_observed, c)
    Q_age = compute_age_quality(update.timestamp, current_time)
    Q_visibility = compute_visibility_quality(update.robot_pose_when_observed, scan, c)
    Q_duplicate = compute_duplicate_quality(j, c, update.claim_id, update.scan_id)
    Q_total = Q_range * Q_age * Q_visibility * Q_duplicate

    R = compute_verification_confidence(update.claim_id, j, c)

    w = T * C * Q_total * R * ramp
    w = clamp(w, 0.0, 1.0)

    O[c] = evidence_alpha * O[c]
    F[c] = evidence_alpha * F[c]

    if update.reported_state == OCCUPIED:
        O[c] = min(occupied_evidence_cap, O[c] + w)
    else if update.reported_state == FREE:
        F[c] = min(free_evidence_cap, F[c] + w)

    store claim contribution for possible later removal

    P_occ[c] = O[c] / (O[c] + F[c] + epsilon)
    total = O[c] + F[c]
    dispute = 2 * min(O[c], F[c]) / (O[c] + F[c] + epsilon)

    if total < theta_unknown_evidence:
        state[c] = UNKNOWN
    else if O[c] > theta_unknown_evidence and F[c] > theta_unknown_evidence and dispute > theta_dispute:
        state[c] = DISPUTED
    else if P_occ[c] > theta_occupied:
        state[c] = OCCUPIED
    else if P_occ[c] < theta_free:
        state[c] = CLEAR
    else:
        state[c] = SUSPICIOUS

    publish shared map, confidence map, and cell state map
    log all intermediate values

for each verification receipt:

    convert receipt to MATE pseudomeasurement rho = (v, c)
    update lifetime and recent MATE trust PDFs

    if receipt contradicts an active claim:
        remove or downgrade the stored claim contribution
```

## 5. Claim-Level Verification Receipts

### 5.1 Purpose

A verification receipt records whether a later physical observation confirmed, contradicted, or could not evaluate a previous map claim.

This is the main difference between simple robot-level trust and the proposed method. Trust is not updated only because a robot has a general reputation. It is updated because a specific map claim was physically checked later.

### 5.2 Map Claim Fields

Each map update should include:

```text
claim_id
reporting_robot_id
target_robot_id
cell_x
cell_y
reported_state
timestamp
robot_pose_when_observed
scan_id
is_attack_report
```

`is_attack_report` is only for simulation and evaluation. It should not be used by the defense logic.

### 5.3 Receipt Fields

Each verification receipt should include:

```text
claim_id
reporting_robot_id
verifying_robot_id
cell_x
cell_y
original_claim_type
verification_result
verification_time
verifier_pose
scan_id
```

Allowed verification results:

```text
CONFIRMED
CONTRADICTED
UNCERTAIN
```

### 5.4 Verification Logic for Fake Obstacles

If the original claim was:

```text
reported_state = OCCUPIED
```

Then when a verifier robot later observes the cell:

```text
if LiDAR shows an obstacle at that cell:
    verification_result = CONFIRMED
elif LiDAR ray passes through the cell as free space:
    verification_result = CONTRADICTED
else:
    verification_result = UNCERTAIN
```

Trust update:

```text
CONFIRMED -> create PSM rho = (1.0, c_verify) and update MATE trust
CONTRADICTED -> create PSM rho = (0.0, c_verify) and update MATE trust
UNCERTAIN -> no trust update
```

### 5.5 Verification Logic for Future Fake Clearing

If the original claim was:

```text
reported_state = FREE
```

Then when a verifier robot later observes the cell:

```text
if LiDAR shows the cell is actually free:
    verification_result = CONFIRMED
elif LiDAR shows an obstacle at that cell:
    verification_result = CONTRADICTED
else:
    verification_result = UNCERTAIN
```

Fake clearing is future work, but the receipt design should support it.

### 5.6 Receipt Timing Variables

For each claim:

```text
t_claim = time when fake or shared claim is published
t_verified = time when a receipt confirms or contradicts the claim
t_removed = time when poisoned data is cleared, downgraded, ignored, or marked suspicious
```

Useful durations:

```text
verification_delay = t_verified - t_claim
poisoned_data_removal_time = t_removed - t_claim
```

---

## 6. Simulation Trial Design

### 6.1 Required Experiment Matrix

First experiment batch:

```text
3 fusion models x 3 maps x 3-5 trials per model/map
```

Fusion models:

```text
log_odds
mate_log_odds
mate_claim_verification
```

Maps:

```text
office
single_hallway
two_path
```

Attack:

```text
fake_obstacle
```

Optional later additions:

```text
small_maze
fake_clearing
late_attacker
noisy_honest_robot
```

### 6.2 Fair Comparison Rules

For each map and trial seed, keep these identical across all three models:

```text
same Webots world
same start pose
same checkpoint route
same robot model
same sensor model
same Nav2 parameters
same attack location
same attacker robot
same attack start time
same attack duration
same random seed if applicable
same success, stuck, and collision definitions
```

Only change:

```text
fusion_mode
```

### 6.3 Recommended Fusion Mode Parameter

```text
fusion_mode = log_odds
fusion_mode = mate_log_odds
fusion_mode = mate_claim_verification
```

Each node should read this parameter and execute the correct map update behavior.

### 6.4 Map-Specific Attack Placement

| Map | Attack placement | Expected effect |
|---|---|---|
| `office` | Doorway, hallway, or common corridor segment | Tests realistic rerouting and map accuracy |
| `single_hallway` | Middle of only route | Tests whether fake obstacle can stop progress |
| `two_path` | Short path between checkpoints | Tests unnecessary detour to long route |

### 6.5 Clean Reference Runs

Before attack trials, run clean no-attack trials to record reference values:

```text
path_length_clean_reference
clean_completion_time
clean_route_taken
clean_final_map_accuracy
```

These reference values are used to compute checkpoint delay and path length increase.

---

## 7. Measurements and Metrics

The project should emphasize two main outcomes:

```text
1. final map accuracy
2. navigation behavior under attack
```

Trust and poisoned-data metrics explain why the navigation behavior changed.

### 7.1 Final Map Accuracy

Purpose:

```text
Measure how close the final shared map is to the ground-truth map.
```

Equation:

```text
final_map_accuracy = correctly_classified_cells / total_evaluated_cells
```

Recommended evaluation area:

```text
Only evaluate cells inside the known map bounds and inside reachable/navigation-relevant space.
```

Avoid counting huge unknown/outside areas, because that can make accuracy misleading.

Cell correctness:

```text
predicted OCCUPIED is correct if ground_truth_state == OCCUPIED
predicted CLEAR is correct if ground_truth_state == FREE
UNKNOWN/SUSPICIOUS/DISPUTED can be counted separately or treated as incorrect depending on analysis goal
```

Recommended first version:

```text
main accuracy counts OCCUPIED and CLEAR only
also report uncertain_cell_rate separately
```

### 7.2 False Occupied Rate

Purpose:

```text
Measure how often free cells are incorrectly marked occupied.
```

This is the main map metric for fake obstacle attacks.

Equation:

```text
false_occupied_rate = ground_truth_free_cells_predicted_occupied / total_ground_truth_free_cells_evaluated
```

For targeted fake obstacle cells:

```text
fake_obstacle_acceptance_rate = fake_obstacle_cells_still_occupied / total_fake_obstacle_cells
```

This metric should be lower for the proposed method.

### 7.3 Final Poisoned Data Remaining

Purpose:

```text
Measure how much of the injected fake obstacle remains in the final map.
```

Equation:

```text
poisoned_data_remaining_rate = poisoned_cells_still_occupied_or_lethal / total_poisoned_cells
```

For Model 3, optionally count `SUSPICIOUS` and `DISPUTED` separately:

```text
poisoned_cells_marked_suspicious_rate = poisoned_cells_suspicious / total_poisoned_cells
poisoned_cells_marked_disputed_rate = poisoned_cells_disputed / total_poisoned_cells
```

### 7.4 Time to Remove Poisoned Data

Purpose:

```text
Measure how quickly the map recovers from fake obstacle insertion.
```

Equation:

```text
poisoned_data_removal_time = t_removed - t_attack_start
```

`t_removed` means the fake obstacle cell is no longer treated as a blocking occupied cell for navigation.

For each method:

```text
log_odds: removed when P_occ falls below occupied threshold
mate_log_odds: removed when P_occ falls below occupied threshold
mate_claim_verification: removed when cell becomes CLEAR, SUSPICIOUS, DISPUTED, or no longer lethal to Nav2
```

If the poison is never removed before trial timeout:

```text
poisoned_data_removal_time = timeout value
removed = false
```

### 7.5 Checkpoint Success Rate

Purpose:

```text
Measure whether the robot completes the checkpoint route under attack.
```

Equation:

```text
checkpoint_success_rate = successful_trials / total_trials
```

A trial is successful if:

```text
robot reaches all required checkpoints
robot does not collide
robot does not get stuck
robot finishes within the time limit
```

### 7.6 Checkpoint Delay / Time to Finish Route

Purpose:

```text
Measure how much fake map data slows navigation.
```

Per trial:

```text
checkpoint_delay = time_to_finish_route_attack - time_to_finish_route_clean_reference
```

If the robot fails:

```text
checkpoint_delay can be set to timeout - clean_reference_time
or reported separately as failure
```

Main field:

```text
time_to_finish_route
```

### 7.7 Path Length Increase

Purpose:

```text
Measure whether the fake obstacle forces unnecessary detours.
```

Equation:

```text
path_length_increase_percent = 100 * (path_length_actual - path_length_clean_reference) / path_length_clean_reference
```

This is especially important in the two-path map.

### 7.8 Reroute Behavior / New Route Chosen

Purpose:

```text
Measure whether the robot changed routes because of poisoned map data.
```

Possible fields:

```text
reroute_taken = true or false
route_choice = short_path, long_path, blocked, unknown
number_of_replans
number_of_recovery_behaviors
```

For the two-path map:

```text
if clean route uses short path and attacked route uses long path:
    reroute_taken = true
    route_choice = long_path
```

For the single-hallway map:

```text
reroute may be impossible, so stuck_detected is more important
```

### 7.9 Stuck Rate

Purpose:

```text
Measure whether fake obstacles cause the robot to stop making progress.
```

Suggested definition:

```text
stuck_detected = true if robot moves less than 0.1 meters over 10 seconds while not at final checkpoint
```

Equation:

```text
stuck_rate = stuck_trials / total_trials
```

### 7.10 Collision Rate

Purpose:

```text
Measure safety failures.
```

For fake obstacle insertion, collisions may not be the main issue. For future fake-clearing attacks, collisions become very important.

Equation:

```text
collision_rate = trials_with_collision / total_trials
```

Collision can be detected by:

```text
Webots contact event
robot body intersects obstacle
minimum distance to real obstacle below safety radius
```

### 7.11 Minimum Clearance

Purpose:

```text
Measure how close the robot comes to real obstacles.
```

This is optional for the first fake-obstacle-only version, but useful later.

Equation:

```text
minimum_clearance = min over time distance(robot_body, nearest_real_obstacle)
```

### 7.12 Attacker Detection Delay

Purpose:

```text
Measure how quickly trust logic identifies the malicious robot.
```

For Model 2 and Model 3:

```text
detection_delay = t_detected - t_first_malicious_report
```

Possible detection definitions:

```text
T_ij drops below detection threshold
or robot status becomes suspicious
or quarantine triggers
```

Recommended threshold:

```text
detection threshold = 0.4
```

Quarantine delay:

```text
quarantine_delay = t_quarantine - t_first_malicious_report
```

### 7.13 False Punishment Rate

Purpose:

```text
Measure whether honest or noisy robots are incorrectly distrusted.
```

This is most relevant for a later noisy-honest-robot scenario.

Equation:

```text
false_punishment_rate = honest_robots_flagged_or_quarantined / total_honest_robots
```

For the first fake-obstacle-only experiment, log this if possible but treat it as secondary.

### 7.14 Runtime Per Update

Purpose:

```text
Measure computational practicality.
```

Equation:

```text
runtime_per_update_ms = total_map_fusion_compute_time_ms / number_of_processed_map_updates
```

Also log:

```text
max_update_runtime_ms
average_update_runtime_ms
95th_percentile_update_runtime_ms
```

The proposed method can be slower than log-odds, but it should remain practical for real-time navigation.

---

## 8. Required Logs

### 8.1 `map_updates.csv`

Recommended fields:

```text
timestamp
trial_id
fusion_mode
map_name
route_name
robot_id
observer_robot_id
reporting_robot_id
target_robot_id
claim_id
cell_x
cell_y
reported_state
ground_truth_state
is_attack_report
attack_type
attack_location_id
L_c_before
L_c_after
P_occ_after
O_c_before
F_c_before
O_c_after
F_c_after
cell_state_after
runtime_ms
```

Fields that do not apply to a model can be blank or `N/A`.

### 8.2 `trust_history.csv`

Recommended fields:

```text
timestamp
trial_id
fusion_mode
map_name
observer_robot_id
reporting_robot_id
alpha_life
beta_life
alpha_recent
beta_recent
T_life
T_recent
T_combined
trust_precision
n_eff
trust_confidence
caution_lambda
update_ramp
quarantined
robot_status
```

For Model 1, trust fields should be `N/A`.

For Model 2, `alpha_life`, `beta_life`, `T_life`, and propagation parameters are required, while confidence/ramp fields can be `N/A`.

For Model 3, all fields should be recorded.

### 8.3 `verification_receipts.csv`

Recommended fields:

```text
timestamp
trial_id
fusion_mode
map_name
claim_id
reporting_robot_id
verifying_robot_id
cell_x
cell_y
original_claim_type
verification_result
verification_time
verifier_pose_x
verifier_pose_y
verifier_pose_yaw
scan_id
R_jc_after
trust_update_applied
```

Model 1 may not use receipts.

Model 2 uses verification receipts only to create MATE pseudomeasurements and update robot trust. It does not use full claim-level map confidence.

Model 3 uses receipts for both MATE trust updates and claim-level map correction or evidence removal.

### 8.4 `navigation_trials.csv`

Recommended fields:

```text
trial_id
fusion_mode
map_name
route_name
random_seed
start_pose_x
start_pose_y
start_pose_yaw
attacker_robot_id
victim_robot_id
attack_type
attack_location_id
attack_start_time
attack_duration
route_completed
checkpoint_success
checkpoints_reached
checkpoints_total
time_to_finish_route
clean_reference_time
checkpoint_delay
path_length_actual
path_length_clean_reference
path_length_increase_percent
reroute_taken
route_choice
number_of_replans
number_of_recovery_behaviors
stuck_detected
collision_detected
minimum_clearance
final_map_accuracy
final_false_occupied_rate
poisoned_data_remaining_rate
poisoned_data_removal_time
attacker_detection_delay
quarantine_delay
runtime_per_update_avg_ms
runtime_per_update_max_ms
```

### 8.5 `trial_events.csv`

Optional but useful for debugging:

```text
timestamp
trial_id
event_type
event_description
robot_id
cell_x
cell_y
claim_id
```

Example event types:

```text
TRIAL_START
ATTACK_STARTED
MAP_UPDATE_RECEIVED
CLAIM_VERIFIED
TRUST_UPDATED
ROBOT_QUARANTINED
ROUTE_REPLANNED
CHECKPOINT_REACHED
STUCK_DETECTED
TRIAL_END
```

---

## 9. Summary Tables to Generate

### 9.1 Main Map and Navigation Table

| Method | Final Map Accuracy | False Occupied Rate | Checkpoint Success | Checkpoint Delay | Path Increase | Runtime |
|---|---:|---:|---:|---:|---:|---:|
| Log-odds | | | | | | |
| MATE-weighted log-odds | | | | | | |
| MATE claim verification | | | | | | |

### 9.2 Attack Defense Table

| Method | Fake Obstacle Acceptance | Poisoned Data Remaining | Poisoned Data Removal Time | Reroute Rate |
|---|---:|---:|---:|---:|
| Log-odds | | | | |
| MATE-weighted log-odds | | | | |
| MATE claim verification | | | | |

### 9.3 Trust Table

| Method | Detection Delay | Quarantine Delay | False Punishment Rate | Final Attacker Trust |
|---|---:|---:|---:|---:|
| Log-odds | N/A | N/A | N/A | N/A |
| MATE-weighted log-odds | | | | |
| MATE claim verification | | | | |

### 9.4 Per-Map Table

| Map | Method | Final Map Accuracy | False Occupied Rate | Checkpoint Delay | Path Increase | Success Rate |
|---|---|---:|---:|---:|---:|---:|
| office | Log-odds | | | | | |
| office | MATE-weighted log-odds | | | | | |
| office | MATE claim verification | | | | | |
| single_hallway | Log-odds | | | | | |
| single_hallway | MATE-weighted log-odds | | | | | |
| single_hallway | MATE claim verification | | | | | |
| two_path | Log-odds | | | | | |
| two_path | MATE-weighted log-odds | | | | | |
| two_path | MATE claim verification | | | | | |

---

## 10. Expected Results and Interpretation

### 10.1 Expected Model 1 Results

Standard log-odds is expected to be most vulnerable to fake obstacle injection because it does not know whether the reporting robot is trustworthy.

Expected symptoms:

```text
higher false occupied rate
higher fake obstacle acceptance
more unnecessary rerouting
longer checkpoint delay
higher stuck rate in single hallway
possibly lower final map accuracy
```

### 10.2 Expected Model 2 Results

MATE-weighted log-odds should improve over Model 1 if the attacker has already been verified as unreliable. Optional MATE-style trust propagation and negative trust updates should help with late attackers. However, Method 2 may still struggle because it lacks Method 3's trust-confidence fusion multiplier, caution ramping, report quality, claim-specific verification confidence, suspicious/disputed states, quarantine, and claim-level evidence removal.

Expected symptoms:

```text
lower false occupied rate than Model 1
some reduction in fake obstacle acceptance
moderate improvement in navigation
possible weakness if attacker starts with neutral or high trust
```

### 10.3 Expected Model 3 Results

The proposed method should perform best because fake obstacle claims start with limited confidence and can be downgraded or removed after physical verification.

Expected symptoms:

```text
lowest false occupied rate
highest final map accuracy
shortest poisoned data removal time
less unnecessary rerouting
lower checkpoint delay
better attacker detection
reasonable runtime overhead
```

### 10.4 Important Interpretation Notes

Do not claim that the method completely prevents map poisoning unless the data clearly proves that. The safer claim is that it reduces the effects of map poisoning.

The strongest evidence will be:

```text
Model 3 has better final map accuracy and navigation outcomes than both Model 1 and Model 2 under the same fake obstacle attacks.
```

If Model 3 improves map accuracy but greatly increases runtime or causes excessive false punishment, that should be reported honestly.

---

## 11. Minimal Implementation Checklist

1. Implement `fusion_mode` with three values:

```text
log_odds
mate_log_odds
mate_claim_verification
```

2. Implement standard log-odds update for Model 1.
3. Implement MATE-weighted log-odds for Model 2.
4. Implement trust-weighted occupied/free evidence fusion for Model 3.
5. Add unique `claim_id` to map updates.
6. Store active unverified claims.
7. Generate verification receipts from later LiDAR observations.
8. Convert confirmed or contradicted receipts into MATE pseudomeasurements and update trust PDFs.
9. Convert map states to Nav2-compatible navigation costs.
10. Run identical routes and attacks across all three models.
11. Log map, trust, verification, navigation, and runtime data.
12. Compute summary metrics by model and map.

---

## 12. Recommended First Trial Batch

Run clean reference trials first:

```text
3 maps x 1-3 clean trials per map
```

Then run attacked trials:

```text
3 models x 3 maps x 3-5 trials per model/map
```

Example trial IDs:

```text
office_log_odds_001
office_mate_log_odds_001
office_mate_claim_verification_001
single_hallway_log_odds_001
single_hallway_mate_log_odds_001
single_hallway_mate_claim_verification_001
two_path_log_odds_001
two_path_mate_log_odds_001
two_path_mate_claim_verification_001
```

For each set of three matching trials, use the same:

```text
map
route
attack location
attack start time
random seed
attacker robot
victim robot
```

---

## 13. Implementation Notes for AI Agent

An AI implementation agent should treat this document as the equation and measurement specification.

The agent should not modify Webots robot physics or controller behavior unless absolutely necessary. The main project logic belongs in ROS-side nodes:

```text
map_merge_node
trust_manager_node
claim_verifier_node
navigation_adapter_node
trial_logger_node
fake_obstacle_injector_node
```

The key deliverables are:

```text
1. three selectable fusion modes
2. claim IDs and verification receipts
3. trust/history updates
4. final map and cell state outputs
5. checkpoint navigation logs
6. summary metrics and tables
```

The main success condition is not only a better-looking map. The defense should improve navigation outcomes under fake obstacle map poisoning while preserving or improving final map accuracy.
