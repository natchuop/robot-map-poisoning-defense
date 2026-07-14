# Equations and Simulation Measurements for Trust-Based Map Poisoning Defense

This document defines the final equations and implementation variables for the three map-fusion methods:

```text
1. log_odds
2. mate_log_odds
3. mate_claim_verification
```

The main attack studied first is fake obstacle insertion:

```text
A compromised robot reports a free cell as OCCUPIED.
```

Fake clearing, where a compromised robot reports a real obstacle as FREE, can be treated as future or secondary work.

---

## 1. Common Design Rules

### 1.1 Direct local LiDAR precedence

All methods use the robot's own current LiDAR observation as the highest-priority evidence for currently visible cells.

```text
If local LiDAR currently sees cell c as FREE:
    clear or strongly downgrade remote OCCUPIED claims for c.

If local LiDAR currently sees cell c as OCCUPIED:
    preserve occupied evidence for c even if remote reports say FREE.

If local LiDAR does not currently observe cell c:
    use the selected fusion method for remote claims.
```

This rule is not a trust rule. It is a sensor-precedence rule.

### 1.2 Shared LiDAR range quality

All three methods use range quality for LiDAR-derived evidence.

The base occupied and free strengths are:

```text
l_occ_base = 0.85
l_free_base = -0.40
```

Method 3 uses positive evidence strengths:

```text
e_occ = 0.85
e_free = 0.40
```

The free evidence constant is positive in Method 3 because free evidence is stored in its own positive bucket `F_c`.

The range-quality function is:

```text
q(r) = 1.0                               if r <= r_near
q(r) = q_min                             if r >= r_far
q(r) = q_min + (1 - q_min) * (1 - s(u))  otherwise

u = (r - r_near) / (r_far - r_near)
s(u) = u^2 * (3 - 2u)
```

Recommended values:

```text
r_near = 1.25 meters
r_far = 4.0 meters
q_min = 0.15
max_free_clear_range = 3.0 meters
```

Interpretation:

```text
near LiDAR reading -> q(r) close to 1.0 -> strong evidence
far LiDAR reading  -> q(r) close to q_min -> weak evidence
```

This is a sensor model, not a trust-defense model.

---

## 2. Common Notation

| Symbol / variable | Meaning |
|---|---|
| `i` | Receiving, observing, or verifying robot |
| `j` | Reporting robot |
| `c` | Occupancy-grid cell |
| `t` | Time or update step |
| `claim_id` | Unique ID for a shared map claim |
| `reported_state` | `OCCUPIED` or `FREE` |
| `P_occ(c)` | Occupancy probability or occupancy-like score |
| `L_c` | Log-odds value for cell `c` |
| `q(r)` | LiDAR range-quality multiplier |
| `O_c` | Occupied evidence for cell `c` in Method 3 |
| `F_c` | Free evidence for cell `c` in Method 3 |
| `epsilon` | Small constant to prevent division by zero |
| `alpha`, `beta` | Beta trust parameters |
| `alpha0`, `beta0` | Neutral prior parameters, usually `1.0`, `1.0` |
| `T_ij` | Trust mean for reporter `j` from observer `i` |
| `nu_ij` | Trust precision, `alpha_ij + beta_ij` |
| `n_eff_ij` | Effective evidence count after subtracting neutral prior precision |
| `C_ij` | Confidence in the trust estimate |
| `Q_ijc` | Report quality for robot `j`'s claim about cell `c` |
| `Q_range_ijc` | Distance-based report quality |
| `Q_visibility_ijc` | Binary visibility quality, `1` or `0` |
| `R_ijc` | Verification confidence for the specific claim |
| `w_ijc` | Final Method 3 influence weight |
| `E_c` | Total evidence, `O_c + F_c` |
| `D_c` | Dispute score |

Recommended constants:

```text
epsilon = 1e-6
L_min = -5.0
L_max = 5.0

theta_occupied = 0.7
theta_free = 0.3

alpha0 = 1.0
beta0 = 1.0
mate_negative_bias = 2.0 or 3.0
mate_negative_threshold = 0.5
mate_min_psm_confidence = 0.2

method2_prior_pull_omega = 0.001 to 0.01
method3_prior_pull_omega = 0.0

recent_window_seconds = 30.0
trust_confidence_k = 10.0

occupied_evidence_cap = 10.0
free_evidence_cap = 10.0
theta_unknown_evidence = 0.5
theta_dispute = 0.5

k_remove = 1.0
k_verify = 1.0
```

Important final simplifications:

```text
Method 3 uses no trust decay.
Method 3 uses no caution ramp.
Method 3 uses no quarantine.
Method 3 uses no duplicate quality multiplier.
Exact duplicate claim IDs should simply be ignored.
Method 3 uses UNCERTAIN, not SUSPICIOUS.
Method 3 uses binary visibility quality: 1 if observed, 0 if not observed.
```

---

## 3. Method 1: Range-Weighted Log-Odds Baseline

### 3.1 Purpose

Method 1 is the full-trust baseline. It uses standard occupancy-grid log-odds fusion and does not use robot trust.

In practice:

```text
T_ij = 1.0 for all reporting robots j
```

It answers:

```text
What happens if shared map claims are accepted without a trust defense?
```

### 3.2 Log-odds cell value

Each cell stores one log-odds value:

```text
L_c(t) = log(P_c(t) / (1 - P_c(t)))
```

Unknown initialization:

```text
P_c(0) = 0.5
L_c(0) = 0.0
```

### 3.3 Range-weighted update

For an observation at range `r`:

```text
l_occ(r) = l_occ_base * q(r)
l_free(r) = l_free_base * q(r)
```

If the report says occupied:

```text
L_c(t+1) = L_c(t) + l_occ_base * q(r)
```

If the report says free:

```text
L_c(t+1) = L_c(t) + l_free_base * q(r)
```

Since `l_free_base = -0.40`, the free update reduces `L_c`.

### 3.4 Probability conversion

```text
P_occ(c) = 1 / (1 + exp(-L_c))
```

Classification:

```text
if P_occ(c) > 0.7:
    state = OCCUPIED
elif P_occ(c) < 0.3:
    state = CLEAR
else:
    state = UNKNOWN
```

### 3.5 Method 1 pseudocode

```text
for each incoming map update about cell c:
    q = compute_range_quality(report_sensor_pose, c)

    if reported_state == OCCUPIED:
        L[c] = L[c] + l_occ_base * q
    else if reported_state == FREE:
        L[c] = L[c] + l_free_base * q

    L[c] = clamp(L[c], L_min, L_max)
    P_occ[c] = 1 / (1 + exp(-L[c]))
    classify using 0.7 and 0.3 thresholds
```

---

## 4. Method 2: MATE-Weighted Range-Weighted Log-Odds

### 4.1 Purpose

Method 2 adds robot-level MATE trust to Method 1. It still uses one log-odds value per cell.

Method 2 asks:

```text
What did the report say, and how much do I trust the reporting robot?
```

It does not use Method 3 features such as trust confidence multiplier, report quality multiplier beyond range quality, verification confidence multiplier, occupied/free evidence layers, uncertain/disputed states, or claim-level evidence removal.

### 4.2 Trust PDF

Each observer robot `i` stores a trust distribution for reporter `j`:

```text
tau_ij(t) ~ Beta(alpha_ij(t), beta_ij(t))
```

Neutral prior:

```text
alpha_ij(0) = alpha0 = 1.0
beta_ij(0) = beta0 = 1.0
```

Trust mean:

```text
T_ij(t) = alpha_ij(t) / (alpha_ij(t) + beta_ij(t))
```

Trust precision:

```text
nu_ij(t) = alpha_ij(t) + beta_ij(t)
```

`T_ij` is used in the Method 2 map update. `nu_ij` is logged or interpreted, but it is not used directly as a Method 2 map multiplier.

### 4.3 Method 2 optional trust decay

Method 2 has only one trust estimate, so optional decay is useful for late attackers.

Before adding a new verification result:

```text
alpha_minus_ij(t) = (1 - omega) * alpha_ij(t-1) + omega * alpha0
beta_minus_ij(t)  = (1 - omega) * beta_ij(t-1)  + omega * beta0
```

Recommended:

```text
omega = 0.001 to 0.01
```

### 4.4 Verification pseudomeasurement

A verification receipt becomes:

```text
rho_ijc(t) = (v_ijc(t), c_ijc(t))
```

where:

```text
v_ijc = 1.0 if the claim was confirmed
v_ijc = 0.0 if the claim was contradicted
c_ijc = confidence in the verification result
```

If the cell was not visible, no trust update is applied.

Negative evidence bias:

```text
omega_neg = mate_negative_bias if v_ijc < mate_negative_threshold else 1.0
```

Trust update:

```text
alpha_ij(t) = alpha_minus_ij(t) + c_ijc(t) * v_ijc(t)
beta_ij(t)  = beta_minus_ij(t)  + omega_neg * c_ijc(t) * (1 - v_ijc(t))
```

### 4.5 Method 2 map update

First compute range quality:

```text
q = q(r)
```

If robot `j` reports occupied:

```text
L_c(t+1) = L_c(t) + T_ij(t) * l_occ_base * q(r)
```

If robot `j` reports free:

```text
L_c(t+1) = L_c(t) + T_ij(t) * l_free_base * q(r)
```

Then:

```text
P_occ(c) = 1 / (1 + exp(-L_c))
```

Method 2 is better than Method 1 because low-trust robots have weaker future influence. However, it still cannot remove old poisoned evidence that was already added to the log-odds map.

---

## 5. Method 3: MATE-Based Claim Verification Defense

### 5.1 Purpose

Method 3 is the proposed defense. It uses robot trust plus claim-level reasoning.

Method 3 asks:

```text
How much should this specific robot's specific claim about this specific cell affect the map right now?
```

Final Method 3 claim weight:

```text
w_ijc(t) = T_ij(t) * C_ij(t) * Q_ijc(t) * R_ijc(t)
```

No ramping. No quarantine. No duplicate quality multiplier. No Method 3 trust decay.

---

## 6. Method 3 Trust: Lifetime and Recent

### 6.1 Lifetime trust

```text
tau_life_ij(t) ~ Beta(alpha_life_ij(t), beta_life_ij(t))
```

```text
T_life_ij(t) = alpha_life_ij(t) / (alpha_life_ij(t) + beta_life_ij(t))
```

Lifetime trust measures long-term behavior across the experiment.

### 6.2 Recent trust

Recent trust should use a time window, not a LiDAR-frame count.

Recommended:

```text
recent_window_seconds = 30.0
```

Only verification events count, not every LiDAR scan frame. A verification event means a previous claim was confirmed or contradicted.

For pseudomeasurements `rho` in the last 30 seconds:

```text
alpha_recent_ij(t) = alpha0 + sum_recent(c_rho * v_rho)
```

```text
beta_recent_ij(t) = beta0 + sum_recent(omega_neg_rho * c_rho * (1 - v_rho))
```

Then:

```text
T_recent_ij(t) = alpha_recent_ij(t) / (alpha_recent_ij(t) + beta_recent_ij(t))
```

### 6.3 Combined trust

```text
T_ij(t) = min(T_life_ij(t), T_recent_ij(t))
```

This catches late attackers. If a robot was trustworthy historically but starts lying recently, recent trust drops and the combined trust also drops.

### 6.4 No Method 3 trust decay

Method 3 uses:

```text
method3_prior_pull_omega = 0.0
```

Reason:

```text
Method 3 already has recent trust to detect behavior changes.
Adding decay would be redundant for the first implementation.
```

---

## 7. Method 3 Trust Updates

A verification result is converted into:

```text
rho_ijc(t) = (v_ijc(t), c_ijc(t))
```

For fake obstacle insertion:

```text
If robot j claimed OCCUPIED and later LiDAR confirms obstacle:
    v_ijc = 1.0

If robot j claimed OCCUPIED and later LiDAR passes through cell as FREE:
    v_ijc = 0.0

If the cell was not visible:
    no trust update
```

Verification confidence for the trust update:

```text
c_ijc = 1.0 for direct reliable LiDAR verification
c_ijc = 0.5 for weaker but usable verification
no update if not visible
```

Negative evidence bias:

```text
omega_neg = mate_negative_bias if v_ijc < mate_negative_threshold else 1.0
```

Lifetime update with no decay:

```text
alpha_life_ij(t) = alpha_life_ij(t-1) + c_ijc(t) * v_ijc(t)
```

```text
beta_life_ij(t) = beta_life_ij(t-1) + omega_neg * c_ijc(t) * (1 - v_ijc(t))
```

Recent trust is recomputed from the last 30 seconds of verification events.

---

## 8. Method 3 Trust Confidence

Trust mean tells what the trust score is:

```text
T_ij = alpha / (alpha + beta)
```

Trust confidence tells how much evidence supports that score.

Use lifetime precision:

```text
nu_life_ij(t) = alpha_life_ij(t) + beta_life_ij(t)
```

Neutral prior precision:

```text
nu0 = alpha0 + beta0
```

Effective evidence count:

```text
n_eff_ij(t) = max(0, nu_life_ij(t) - nu0)
```

Reason for subtracting `nu0`:

```text
alpha0 and beta0 are the starting assumption, not real observations.
Subtracting nu0 removes that artificial prior evidence.
```

Trust confidence:

```text
C_ij(t) = n_eff_ij(t) / (n_eff_ij(t) + trust_confidence_k)
```

Recommended:

```text
trust_confidence_k = 10.0
```

Interpretation:

```text
n_eff = 0  -> C_ij = 0
n_eff = 10 -> C_ij = 0.5
n_eff = 30 -> C_ij = 0.75
large n_eff -> C_ij approaches 1.0
```

Use lifetime precision for `C_ij`, not recent precision, because recent precision can fluctuate heavily in a short trial. Recent trust already handles recent behavior changes. Lifetime precision is better for deciding whether the robot has enough verified history to make the trust estimate meaningful.

---

## 9. Method 3 Report Quality

Report quality means:

```text
How reliable was this specific LiDAR-based observation of this specific cell?
```

Final simplified report quality:

```text
Q_ijc(t) = Q_range_ijc(t) * Q_visibility_ijc(t)
```

No age term. No duplicate term.

### 9.1 Range quality

Use the shared range-quality function `q(r)`:

```text
Q_range_ijc(t) = q(r_jc(t))
```

where:

```text
r_jc(t) = distance from robot j's observation pose to the center of cell c
```

Meaning:

```text
near cell -> higher Q_range
far cell  -> lower Q_range
```

This measures distance-based sensor reliability.

### 9.2 Visibility quality

Keep visibility simple:

```text
Q_visibility_ijc(t) = 1.0 if a LiDAR ray touches or traverses cell c
Q_visibility_ijc(t) = 0.0 otherwise
```

Examples:

```text
LiDAR endpoint lands in cell c as an obstacle hit -> Q_visibility = 1.0
LiDAR ray passes through cell c as free space -> Q_visibility = 1.0
Cell c is behind an obstacle -> Q_visibility = 0.0
Cell c is outside field of view or range -> Q_visibility = 0.0
Cell c is near a ray but the ray does not enter the cell -> Q_visibility = 0.0
```

Do not count nearby misses or number of nearby rays in the first implementation. A cell is visible only if it is actually touched by the ray-tracing algorithm.

---

## 10. Method 3 Verification Confidence

Verification confidence is claim-specific:

```text
R_ijc(t) = how much this specific claim has been verified
```

Recommended values:

| Verification condition | `R_ijc` |
|---|---:|
| Directly confirmed by this robot's LiDAR | 1.0 |
| Confirmed by another trusted robot | 0.8 |
| Consistent with existing verified map | 0.6 |
| New but unverified claim | 0.3 |
| Conflicts with trusted evidence | 0.0 to 0.2 |

For a new remote occupied claim:

```text
R_ijc = 0.3
```

This prevents a new unverified fake obstacle from immediately becoming a strong map obstacle.

---

## 11. Method 3 Final Influence Weight

```text
w_ijc(t) = T_ij(t) * C_ij(t) * Q_ijc(t) * R_ijc(t)
```

Expanded:

```text
w_ijc(t) = min(T_life_ij(t), T_recent_ij(t))
           * C_ij(t)
           * Q_range_ijc(t)
           * Q_visibility_ijc(t)
           * R_ijc(t)
```

Clamp:

```text
w_ijc(t) = max(0.0, min(w_ijc(t), 1.0))
```

A claim has strong influence only if:

```text
robot trust is high
trust confidence is high
the report was physically reliable
the cell was actually visible
the claim has verification support
```

---

## 12. Method 3 Occupied and Free Evidence Layers

Method 3 stores two positive evidence values per cell:

```text
O_c(t) = occupied evidence for cell c
F_c(t) = free evidence for cell c
```

These are not robot-trust values. They are map-cell evidence values.

Robot trust answers:

```text
Do I trust robot j?
```

Cell evidence answers:

```text
How much evidence says this cell is occupied?
How much evidence says this cell is free?
```

Occupied and free evidence are weighted differently, consistent with Methods 1 and 2.

```text
e_occ = 0.85
e_free = 0.40
```

If the report says occupied:

```text
O_c(t+1) = min(occupied_evidence_cap, O_c(t) + e_occ * w_ijc(t))
F_c(t+1) = F_c(t)
```

If the report says free:

```text
F_c(t+1) = min(free_evidence_cap, F_c(t) + e_free * w_ijc(t))
O_c(t+1) = O_c(t)
```

Recommended caps:

```text
occupied_evidence_cap = 10.0
free_evidence_cap = 10.0
```

Why occupied is stronger:

```text
Obstacle evidence should be cautious.
A possible obstacle should affect the map more than a single free-space claim.
This matches log-odds behavior where +0.85 has larger magnitude than -0.40.
```

---

## 13. Occupancy Score, Dispute Score, and Classification

Occupancy score:

```text
P_occ(c) = O_c / (O_c + F_c + epsilon)
```

where:

```text
epsilon = 1e-6
```

`epsilon` only prevents division by zero.

Total evidence:

```text
E_c = O_c + F_c
```

Dispute score:

```text
D_c = 2 * min(O_c, F_c) / (O_c + F_c + epsilon)
```

Interpretation:

```text
D_c close to 0 -> evidence mostly points one way
D_c close to 1 -> strong occupied and free evidence both exist
```

Classification:

```text
if E_c < theta_unknown_evidence:
    state = UNKNOWN

else if O_c > theta_unknown_evidence and F_c > theta_unknown_evidence and D_c > theta_dispute:
    state = DISPUTED

else if P_occ(c) > theta_occupied:
    state = OCCUPIED

else if P_occ(c) < theta_free:
    state = CLEAR

else:
    state = UNCERTAIN
```

Recommended thresholds:

```text
theta_unknown_evidence = 0.5
theta_occupied = 0.7
theta_free = 0.3
theta_dispute = 0.5
```

Navigation behavior can remain simple:

| Internal state | Meaning | Navigation behavior |
|---|---|---|
| `CLEAR` | Strong free evidence | Free/normal traversal |
| `OCCUPIED` | Strong obstacle evidence | Blocked/lethal cost |
| `UNKNOWN` | Not enough evidence | Normal Nav2 unknown behavior |
| `UNCERTAIN` | Weak or inconclusive evidence | Treat like `UNKNOWN`, log separately |
| `DISPUTED` | Strong occupied and free evidence both exist | Treat like `UNKNOWN`, log separately |

---

## 14. Claim-Level Evidence Storage and Removal

For every accepted Method 3 claim, store:

```text
claim_id
reporting_robot_id
cell_x
cell_y
reported_state
evidence_added = e_occ * w_ijc if OCCUPIED, or e_free * w_ijc if FREE
active = true
```

If an occupied claim is later contradicted:

```text
O_c = max(0, O_c - k_remove * evidence_added)
F_c = min(free_evidence_cap, F_c + k_verify * c_verify * e_free)
active = false
```

If a free claim is later contradicted:

```text
F_c = max(0, F_c - k_remove * evidence_added)
O_c = min(occupied_evidence_cap, O_c + k_verify * c_verify * e_occ)
active = false
```

Recommended:

```text
k_remove = 1.0
k_verify = 1.0
```

This is one of the biggest differences from Method 2. Method 2 can lower a bad reporter's future trust, but old poisoned log-odds evidence may remain. Method 3 can remove or downgrade the specific claim contribution.

---

## 15. Method 3 Pseudocode

```text
for each incoming map update from robot j about cell c:

    # 1. Compute lifetime trust.
    T_life = alpha_life[j] / (alpha_life[j] + beta_life[j])

    # 2. Recompute recent trust from verification events in the last 30 seconds.
    alpha_recent = alpha0 + sum(c_rho * v_rho for rho in recent_window_30s[j])
    beta_recent = beta0 + sum(omega_neg_rho * c_rho * (1 - v_rho) for rho in recent_window_30s[j])
    T_recent = alpha_recent / (alpha_recent + beta_recent)

    # 3. Combine trust cautiously.
    T = min(T_life, T_recent)

    # 4. Compute trust confidence from lifetime precision.
    nu_life = alpha_life[j] + beta_life[j]
    nu0 = alpha0 + beta0
    n_eff = max(0, nu_life - nu0)
    C = n_eff / (n_eff + trust_confidence_k)

    # 5. Compute report quality.
    Q_range = compute_range_quality(report_sensor_pose, c)
    Q_visibility = 1.0 if lidar_ray_touches_cell(report_scan, c) else 0.0
    Q = Q_range * Q_visibility

    # 6. Compute claim-specific verification confidence.
    R = compute_verification_confidence(claim_id, j, c)

    # 7. Compute final influence weight.
    w = T * C * Q * R
    w = clamp(w, 0.0, 1.0)

    # 8. Update occupied/free evidence with different evidence strengths.
    if reported_state == OCCUPIED:
        evidence_added = e_occ * w
        O[c] = min(occupied_evidence_cap, O[c] + evidence_added)
    else if reported_state == FREE:
        evidence_added = e_free * w
        F[c] = min(free_evidence_cap, F[c] + evidence_added)

    # 9. Store contribution for possible later removal.
    store claim_id, robot_id, cell, reported_state, evidence_added, active = true

    # 10. Classify the cell.
    P_occ = O[c] / (O[c] + F[c] + epsilon)
    E = O[c] + F[c]
    D = 2 * min(O[c], F[c]) / (O[c] + F[c] + epsilon)

    if E < theta_unknown_evidence:
        state[c] = UNKNOWN
    else if O[c] > theta_unknown_evidence and F[c] > theta_unknown_evidence and D > theta_dispute:
        state[c] = DISPUTED
    else if P_occ > theta_occupied:
        state[c] = OCCUPIED
    else if P_occ < theta_free:
        state[c] = CLEAR
    else:
        state[c] = UNCERTAIN

for each verification receipt about a previous claim from robot j:

    convert receipt to rho = (v, c_verify)

    if receipt is confirmed or contradicted:
        omega_neg = mate_negative_bias if v < mate_negative_threshold else 1.0
        alpha_life[j] = alpha_life[j] + c_verify * v
        beta_life[j] = beta_life[j] + omega_neg * c_verify * (1 - v)
        add rho to robot j's recent 30-second verification window

    if receipt contradicts an active claim:
        remove or downgrade that stored claim contribution
```

---

## 16. Key Limitations of Method 2

Method 2 is useful, but limited:

```text
1. It uses robot-level trust only, not claim-level verification confidence.
2. It does not use trust confidence C_ij, so a weakly supported trust score can still affect the map.
3. It does not store O_c and F_c, so it cannot distinguish UNKNOWN from UNCERTAIN or DISPUTED.
4. It cannot remove a specific fake obstacle contribution after contradiction.
5. It handles late attackers more slowly than Method 3 because it does not combine lifetime and recent trust.
```

