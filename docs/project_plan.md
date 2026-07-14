# Project Plan: Trust-Based Map Confidence for Robot-to-Robot Map Poisoning Defense

## 1. Project Goal

The goal is to build and test a decentralized trust-based defense for multi-robot mapping and navigation. Multiple robots share map updates while moving through the same environment. One robot may become compromised and publish fake obstacle claims, such as reporting that a clear hallway or doorway is blocked.

The main claim is:

```text
Can decentralized trust-weighted map fusion reduce the effects of map-poisoning attacks on multi-robot navigation and final map accuracy?
```

The system should not blindly accept shared map updates. Each robot should decide how much influence another robot's claim has by combining:

```text
robot trust
trust confidence
LiDAR/report quality
verification confidence
map-cell evidence
```

The project focuses on decentralized trust. Each robot keeps its own trust records and map confidence values. No central server decides which robot is trustworthy.

---

## 2. Research Question

Main question:

```text
Can robots use trust and later LiDAR verification to decide whether shared obstacle reports are real, fake, or uncertain?
```

The comparison separates three questions:

```text
1. What happens when robots trust all shared map updates equally?
2. Does MATE-style robot trust improve over no trust?
3. Does claim-level verification improve beyond robot-level trust?
```

---

## 3. Models to Compare

```text
Model 1: log_odds
Model 2: mate_log_odds
Model 3: mate_claim_verification
```

All three models now use the same basic LiDAR range-quality sensor model. Nearby sensor readings have stronger influence than far readings.

---

## 4. Model 1: Range-Weighted Log-Odds Baseline

Model 1 is the full-trust baseline. It uses normal occupancy-grid log-odds fusion and does not use robot identity or robot trust.

Each cell stores:

```text
L_c(t) = log(P_c(t) / (1 - P_c(t)))
```

Unknown start:

```text
P_c(0) = 0.5
L_c(0) = 0.0
```

All methods use LiDAR range quality:

```text
l_occ(r) = l_occ_base * q(r)
l_free(r) = l_free_base * q(r)
```

Recommended:

```text
l_occ_base = 0.85
l_free_base = -0.40
```

If a report says occupied:

```text
L_c(t+1) = L_c(t) + l_occ_base * q(r)
```

If a report says free:

```text
L_c(t+1) = L_c(t) + l_free_base * q(r)
```

Probability:

```text
P_occ(c) = 1 / (1 + exp(-L_c))
```

Classification:

```text
P_occ(c) > 0.7 -> OCCUPIED
P_occ(c) < 0.3 -> CLEAR
otherwise      -> UNKNOWN
```

This model tests what happens when robots accept shared occupancy updates without a trust defense.

---

## 5. Model 2: MATE-Weighted Log-Odds Baseline

Model 2 adds robot-level trust but keeps the ordinary log-odds map.

Each observing robot `i` keeps a MATE-style trust PDF for each reporting robot `j`:

```text
tau_ij(t) ~ Beta(alpha_ij(t), beta_ij(t))
```

Neutral prior:

```text
alpha0 = 1.0
beta0 = 1.0
T_ij(0) = 0.5
```

Trust mean:

```text
T_ij(t) = alpha_ij(t) / (alpha_ij(t) + beta_ij(t))
```

Trust precision:

```text
nu_ij(t) = alpha_ij(t) + beta_ij(t)
```

Method 2 uses optional trust decay because it has only one trust estimate:

```text
alpha_minus_ij(t) = (1 - omega) * alpha_ij(t-1) + omega * alpha0
beta_minus_ij(t)  = (1 - omega) * beta_ij(t-1)  + omega * beta0
```

Recommended:

```text
omega = 0.001 to 0.01
```

Verification receipts become trust pseudomeasurements:

```text
rho_ijc(t) = (v_ijc(t), c_ijc(t))
```

where:

```text
v_ijc = 1.0 for confirmed claims
v_ijc = 0.0 for contradicted claims
c_ijc = confidence in that verification
```

Trust update:

```text
alpha_ij(t) = alpha_minus_ij(t) + c_ijc(t) * v_ijc(t)
beta_ij(t)  = beta_minus_ij(t)  + omega_neg * c_ijc(t) * (1 - v_ijc(t))
```

Method 2 map fusion:

```text
if robot j reports occupied:
    L_c = L_c + T_ij * l_occ_base * q(r)

if robot j reports free:
    L_c = L_c + T_ij * l_free_base * q(r)
```

This model tests whether robot-level trust alone is enough to reduce fake obstacle acceptance.

---

## 6. Model 3: MATE-Based Claim Verification Defense

Model 3 is the proposed defense. It keeps MATE-style trust but adds claim-level reasoning.

The main idea:

```text
MATE trust decides how reliable the source is.
Claim-level verification decides how much this specific cell claim should affect the map now.
```

Final claim weight:

```text
w_ijc(t) = T_ij(t) * C_ij(t) * Q_ijc(t) * R_ijc(t)
```

where:

```text
T_ij(t) = combined robot trust
C_ij(t) = confidence in that trust estimate
Q_ijc(t) = quality of this report about this cell
R_ijc(t) = verification confidence for this claim
w_ijc(t) = final influence of the claim
```

Final simplifications:

```text
No Method 3 decay.
No caution ramping.
No quarantine.
No duplicate weighting.
Use UNCERTAIN instead of SUSPICIOUS.
Use binary visibility quality: 1 if observed, 0 if not observed.
```

---

## 7. Method 3 Trust Design

Method 3 uses lifetime trust and recent trust.

Lifetime trust:

```text
T_life_ij(t) = alpha_life_ij(t) / (alpha_life_ij(t) + beta_life_ij(t))
```

Recent trust uses a time window, not number of LiDAR frames:

```text
recent_window_seconds = 30.0
```

Only verification events count. A LiDAR scan frame is not automatically a new trust event.

Recent trust:

```text
alpha_recent_ij(t) = alpha0 + sum_recent(c_rho * v_rho)
beta_recent_ij(t)  = beta0 + sum_recent(omega_neg_rho * c_rho * (1 - v_rho))
T_recent_ij(t) = alpha_recent_ij(t) / (alpha_recent_ij(t) + beta_recent_ij(t))
```

Combined trust:

```text
T_ij(t) = min(T_life_ij(t), T_recent_ij(t))
```

This catches late attackers because recent trust drops faster than lifetime trust.

Method 3 uses no trust decay:

```text
method3_prior_pull_omega = 0.0
```

---

## 8. Trust Confidence

Trust confidence measures how much evidence supports the trust estimate.

Lifetime precision:

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

Trust confidence:

```text
C_ij(t) = n_eff_ij(t) / (n_eff_ij(t) + trust_confidence_k)
```

Recommended:

```text
trust_confidence_k = 10.0
```

`T_ij` says how trusted the robot seems. `C_ij` says how much verified evidence supports that trust score.

---

## 9. Report Quality and Verification Confidence

Report quality means:

```text
How reliable was this specific LiDAR-based observation of this cell?
```

Final report quality:

```text
Q_ijc(t) = Q_range_ijc(t) * Q_visibility_ijc(t)
```

Range quality:

```text
Q_range_ijc(t) = q(r_jc(t))
```

where `r_jc(t)` is the distance from robot `j`'s observation pose to the center of cell `c`.

Visibility quality is binary:

```text
Q_visibility_ijc(t) = 1.0 if a LiDAR ray touches or traverses cell c
Q_visibility_ijc(t) = 0.0 otherwise
```

Examples:

```text
LiDAR endpoint hits cell c -> visible
LiDAR ray passes through cell c as free -> visible
cell behind obstacle -> not visible
cell outside range/FOV -> not visible
ray passes nearby but not through cell -> not visible
```

Verification confidence:

| Condition | `R_ijc` |
|---|---:|
| Directly confirmed by LiDAR | 1.0 |
| Confirmed by another trusted robot | 0.8 |
| Consistent with verified map | 0.6 |
| New but unverified claim | 0.3 |
| Conflicts with trusted evidence | 0.0 to 0.2 |

---

## 10. Method 3 Cell Evidence

Method 3 stores two positive evidence values per cell:

```text
O_c(t) = occupied evidence for cell c
F_c(t) = free evidence for cell c
```

These are different from robot trust. Robot trust is about the source. `O_c` and `F_c` are about the cell.

Occupied and free evidence use different strengths:

```text
e_occ = 0.85
e_free = 0.40
```

If a claim says occupied:

```text
O_c(t+1) = min(occupied_evidence_cap, O_c(t) + e_occ * w_ijc(t))
F_c(t+1) = F_c(t)
```

If a claim says free:

```text
F_c(t+1) = min(free_evidence_cap, F_c(t) + e_free * w_ijc(t))
O_c(t+1) = O_c(t)
```

Recommended caps:

```text
occupied_evidence_cap = 10.0
free_evidence_cap = 10.0
```

Occupancy score:

```text
P_occ(c) = O_c / (O_c + F_c + epsilon)
```

Total evidence:

```text
E_c = O_c + F_c
```

Dispute score:

```text
D_c = 2 * min(O_c, F_c) / (O_c + F_c + epsilon)
```

Cell classification:

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

Navigation behavior can stay simple:

```text
CLEAR -> free/normal traversal
OCCUPIED -> blocked/lethal cost
UNKNOWN -> normal Nav2 unknown behavior
UNCERTAIN -> treat like UNKNOWN, log separately
DISPUTED -> treat like UNKNOWN, log separately
```

---

## 11. Claim-Level Evidence Removal

For every accepted Method 3 claim, store:

```text
claim_id
reporting_robot_id
cell_x
cell_y
reported_state
evidence_added
active = true
```

If the claim said occupied and is later contradicted:

```text
O_c = max(0, O_c - k_remove * evidence_added)
F_c = min(free_evidence_cap, F_c + k_verify * c_verify * e_free)
```

If the claim said free and is later contradicted:

```text
F_c = max(0, F_c - k_remove * evidence_added)
O_c = min(occupied_evidence_cap, O_c + k_verify * c_verify * e_occ)
```

Recommended:

```text
k_remove = 1.0
k_verify = 1.0
```

This lets Method 3 remove or downgrade the specific poisoned evidence that was previously added.

---

## 12. Test Environments

Required maps:

```text
office
single_hallway
two_path
```

Optional stress-test map:

```text
small_maze
```

---

## 13. Attack Model

Primary attack:

```text
fake obstacle insertion
```

A compromised robot reports a free cell as occupied.

Example:

```text
real state: FREE
attacker report: OCCUPIED
```

Expected effect:

```text
single_hallway -> may block only route
two_path -> may force longer route
office -> may cause unnecessary rerouting
```

---

## 14. Metrics

Useful metrics:

| Metric | Purpose |
|---|---|
| False occupied rate | Measures how many free cells became falsely occupied |
| Poisoned cells remaining | Measures how much poisoned map evidence remains |
| Time to remove poisoned data | Measures how quickly fake evidence is corrected |
| Path length increase | Measures navigation inefficiency |
| Reroute count | Measures how often fake obstacles changed plans |
| Navigation delay | Measures time lost from fake obstacles |
| Final map accuracy | Measures final occupancy-map correctness |
| Trust recovery/detection delay | Measures how quickly attacker trust drops |
| Uncertain/disputed count | Measures how often Method 3 detects weak or conflicting evidence |

---

## 15. What Method 3 Does That Method 2 Does Not

```text
1. Method 3 uses trust confidence C_ij, so weakly supported trust scores have limited influence.
2. Method 3 uses claim-level report quality Q_ijc and verification confidence R_ijc.
3. Method 3 separates occupied and free evidence into O_c and F_c, allowing UNCERTAIN and DISPUTED states.
4. Method 3 stores each claim contribution and can remove or downgrade poisoned evidence later.
5. Method 3 uses lifetime plus recent trust, so late attackers are caught faster than with one trust estimate.
```

