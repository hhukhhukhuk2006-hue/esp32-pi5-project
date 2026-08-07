import serial
import time

# Khởi tạo kết nối Serial. 
# Lưu ý: Cổng của Arduino trên Pi thường là /dev/ttyACM0 hoặc /dev/ttyUSB0
# Nếu chạy báo lỗi không tìm thấy cổng, bro mở terminal Pi gõ 'ls /dev/ttyA*' hoặc 'ls /dev/ttyU*' để check lại tên cổng nhé.
arduino_port = '/dev/ttyACM0' 
baud_rate = 9600

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    print(f"Đã kết nối với Arduino ở cổng {arduino_port}")
    
    # Đợi 2 giây để Arduino reset và ổn định sau khi mở kết nối Serial
    time.sleep(2) 

    while True:
        # Lấy lệnh từ bàn phím của bro
        lenh = input("Nhập 'o' để bật, 'i' để tắt (nhập 'q' để thoát): ").strip().lower()
        
        if lenh == 'q':
            print("Đã thoát chương trình.")
            break
        elif lenh in ['o', 'i']:
            # Gửi lệnh xuống Arduino (phải encode sang dạng byte)
            ser.write(lenh.encode())
            
            # Đợi một xíu cho Arduino xử lý và gửi phản hồi
            time.sleep(0.1)
            
            # Đọc phản hồi từ Arduino gửi lên
            if ser.in_waiting > 0:
                phan_hoi = ser.readline().decode('utf-8').strip()
                print(f"Arduino phản hồi: {phan_hoi}\n")
        else:
            print("Lệnh không hợp lệ, vui lòng nhập lại!\n")

except serial.SerialException as e:
    print(f"Lỗi kết nối Serial: {e}")
    print("Bro nhớ check lại xem đã cắm cáp chưa hoặc đúng tên cổng /dev/tty... chưa nhé!")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()