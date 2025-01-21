# MULTITASKING SCHEDULER

### RMS (RATE MONOTONIC SCHEDULING)

> - à một thuật toán lập lịch thời gian thực, sử dụng ưu tiên tĩnh (static priority), trong đó các nhiệm vụ (task) được ưu tiên dựa trên chu kỳ (period) của chúng. RMS thường được sử dụng trong các hệ thống thời gian thực cứng (hard real-time systems) để đảm bảo các nhiệm vụ đáp ứng đúng hạn (deadline).

### ROUND ROBIN

> - Trong freeRTOS, khi các task có mức ưu tiên giống nhau, thì chương trình sẽ chạy theo bộ lập lịch Round Robin, với time_quantum = 1/TickRateHZ
