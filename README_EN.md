[🇹🇷 Türkçe versiyon için tıklayın](README.md)

# Sensor-Fusion-and-State-Estimation-Project
Sensor Fusion and State Estimation using Extended Kalman Filter (EKF)

## ABSTRACT
In this study, a UAV flight of approximately 10 minutes was simulated to include different phases such as takeoff, climb, cruise, and landing. The ground truth time series generated from this scenario were used as references for sensor modeling and filter validation. Subsequently, IMU, GNSS, magnetometer, barometer, and pitot tube sensors were modeled to produce asynchronous measurements with realistic noise and bias parameters due to different sampling rates. The related noise and bias characteristics were derived from literature and low-cost sensor specifications available on the market.

On this framework, an Extended Kalman Filter (EKF) was designed and implemented in the NED coordinate frame. The prediction and update steps were applied, and the state transition functions and corresponding Jacobians were formulated. The initial covariance (P₀), process noise covariance (Q), and measurement noise covariance (R) were defined. The innovation magnitude was monitored to evaluate the filter’s stability and consistency. The error covariance update was implemented in both Joseph and simplified forms for comparison. Conditional updates were designed for various field scenarios (e.g., GNSS outage). Bias estimation was integrated into the state vector and tracked over time. Different mathematical forms for updating the covariance matrix (P) were tested.

Finally, the outputs were visualized and quantitatively analyzed. The evolution of the P and Q matrices and the innovation statistics were reported. For attitude estimation, Complementary Filter (CF), Madgwick, and Kalman-based approaches were compared on the same dataset, and their strengths and weaknesses were discussed. The study concludes with potential improvements and future development opportunities.

---

<p align="center">
  <img width="600" alt="3D flight trajectory" src="https://github.com/user-attachments/assets/99237140-8b4d-4e34-9283-0731403d1587" /><br>
  <em>Figure 1.  3D visualization of the flight trajectory with phase annotations</em>
</p>

<p align="center">
  <img width="600" alt="sensor-fusion-block-diagram" src="https://github.com/user-attachments/assets/9dd52b5f-2969-499e-bf58-2230756dfbc4" /><br>
  <em>Figure 2. Flight phases and durations</em>
</p>

---

<div align="center">
<table>
<tr>
<td align="center">
<img src="https://github.com/user-attachments/assets/334aeb74-0290-43e3-a052-f91a15dbf922" width="450"><br>
<img src="https://github.com/user-attachments/assets/dbe2ad96-43fd-4828-8880-aa11873cd315" width="450"><br>
<em>Figure 3. Route, altitude, and horizontal velocity information</em>
</td>
<td align="center">
<img src="https://github.com/user-attachments/assets/01ddea3f-a40e-45b3-847b-796e8ca8dff1" width="450"><br>
<img src="https://github.com/user-attachments/assets/74841a30-8f47-4034-9c74-bcceb23b9463" width="450"><br>
<em>Figure 4. Attitude angle animation and variations in roll, pitch, and yaw</em>
</td>
</tr>
</table>
</div>

---

<div align="center">
<table>
  <thead>
    <tr>
      <th>Sensor</th>
      <th>Output(s)</th>
      <th>Rate (Hz)</th>
      <th>Noise (std)</th>
      <th>Bias/Drift</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Accelerometer</td>
      <td>a<sub>x</sub>, a<sub>y</sub>, a<sub>z</sub></td>
      <td>200</td>
      <td>σ<sub>a</sub> [m/s²]</td>
      <td>b<sub>a</sub> (offset), RW</td>
    </tr>
    <tr>
      <td>Gyroscope</td>
      <td>ω<sub>x</sub>, ω<sub>y</sub>, ω<sub>z</sub></td>
      <td>200</td>
      <td>σ<sub>ω</sub> [rad/s]</td>
      <td>b<sub>ω</sub> (offset), RW</td>
    </tr>
    <tr>
      <td>Magnetometer</td>
      <td>m<sub>x</sub>, m<sub>y</sub>, m<sub>z</sub></td>
      <td>50</td>
      <td>σ<sub>m</sub> [µT]</td>
      <td>-</td>
    </tr>
    <tr>
      <td>Barometer (altimeter)</td>
      <td>p (pressure) / h (altitude)</td>
      <td>25</td>
      <td>σ<sub>p</sub> [Pa] / σ<sub>h</sub> [m]</td>
      <td>-</td>
    </tr>
    <tr>
      <td>GNSS</td>
      <td>p, v</td>
      <td>5</td>
      <td>σ<sub>pos</sub> [m], σ<sub>vel</sub> [m/s]</td>
      <td>-</td>
    </tr>
    <tr>
      <td>Pitot (air speed)</td>
      <td>q (dyn. pressure) / V (air speed)</td>
      <td>25</td>
      <td>σ<sub>q</sub> [Pa], σ<sub>V</sub> [m/s]</td>
      <td>-</td>
    </tr>
  </tbody>
</table>
<p style="margin-top:6px; font-size:12px;">
Table: Asynchronous measurement model, sensors, and sampling frequencies.
</p>
</div>

---

## 15-State EKF Vector

<p align="center">
  <img width="1000" alt="15-state-ekf" src="https://github.com/user-attachments/assets/9e8db22c-f0c2-4ad2-a6bb-7566cea5dbce" /><br>
  <em>Figure 5. 15-state vector structure defined for the Extended Kalman Filter (EKF)</em>
</p>

---

## GNSS Outage – Barometer and Pitot Disabled

<p align="center">
  <img width="330" alt="position_no_baro_pitot" src="https://github.com/user-attachments/assets/51cf6c10-6b06-4670-9f90-36eb513a84f7">
  <img width="330" alt="velocity_no_baro_pitot" src="https://github.com/user-attachments/assets/97305094-bb5c-4607-be11-fdda57334bc9">
  <img width="330" alt="attitude_no_baro_pitot" src="https://github.com/user-attachments/assets/582a9055-a784-4e82-9474-e80c692cce5f"><br>
  <em>Figure 6. Position, velocity, and attitude estimations during GNSS outage with barometer and pitot sensors disabled</em>
</p>

---

## GNSS Outage – Barometer and Pitot Active

<p align="center">
  <img width="330" alt="position_baro_pitot_active" src="https://github.com/user-attachments/assets/5fedd403-dc77-4d02-baf8-a3a396c3a7e9">
  <img width="330" alt="velocity_baro_pitot_active" src="https://github.com/user-attachments/assets/5b6ee86d-ccf9-4c1b-b1e8-0f1cecf08541">
  <img width="330" alt="attitude_baro_pitot_active" src="https://github.com/user-attachments/assets/2340df70-1f57-4dc4-9c70-b7ae192c259f"><br>
  <em>Figure 7. Position, velocity, and attitude estimations during GNSS outage with barometer and pitot sensors active</em>
</p>

---

## Comparison of Filters – Attitude Angles

<p align="center">
<img width="588" height="500" alt="image" src="https://github.com/user-attachments/assets/bc53979b-fe8b-4a4c-b055-3b097a057720" /><br>
  <em>Figure 8. Comparison of EKF, Madgwick, and Complementary filters on attitude estimation</em>
</p>
