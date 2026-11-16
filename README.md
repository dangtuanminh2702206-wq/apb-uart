# APB–UART — bản RTL hoàn thiện ngày 16/11/2026

Nguồn được đọc lại từ thư mục Drive người dùng cung cấp ngày 16/11/2026. Tám file tải về khớp SHA256 của lần rà ngày 15/11. File `reports/source_manifest.json` lưu nguồn và mã kiểm tra. Phiên bản này dùng đặc tả `HD-APB-UART-tuong-tac.html` 1.1.

## Sử dụng

- Add toàn bộ 8 file `.v` trong `rtl` vào công cụ thiết kế. `rtl/files.f` là danh sách file, dùng từ thư mục gốc này.
- Chọn top `apb_uart` khi kiểm tra IP qua bus APB.
- Chọn top `tt_um_hust_apb_uart` khi tích hợp wrapper Tiny Tapeout và cầu SPI–APB.
- RTL dùng Verilog-2001. Thư mục `verification` chứa mã mô phỏng riêng và chỉ dùng trong simulator.
- Chạy `python verify.py` để lặp lại kiểm tra. Mặc định công cụ nằm ở `C:\msys64\ucrt64\bin`. Biến môi trường `RTL_TOOL_BIN` cho phép đổi thư mục công cụ.

## Các sửa đổi

| Khối | Sửa đổi và mục đích |
|---|---|
| APB | Chặn ACCESS bằng `presetn`, bảo đảm PSLVERR và PRDATA bằng 0 trong hardware reset. |
| APB E3 | Ghi TXDATA khi TX bận báo lỗi với mọi PSTRB khác 0, gồm cả byte lane cao. |
| Reset mềm | Dùng chuỗi dịch `11 → 01 → 00` để giữ reset hai chu kỳ. Tránh chuyển đồng thời hai bit theo chiều ngược nhau như bộ đếm `10 → 01`. |
| Giải mã | Bỏ kiểm tra alignment lặp lại vì so sánh chính xác bảy địa chỉ đã bao hàm alignment. Thêm default cho case ghi. |
| TX | Thanh ghi giữ bit dữ liệu đầu tiên có độ rộng 1 bit đúng với nhu cầu sử dụng. Các bit còn lại nằm trong shift register. |
| RX | Index chọn thanh ghi 8 bit dùng đúng 3 bit. Đánh dấu chuỗi đồng bộ RX và CTS bằng thuộc tính `async_reg`. |
| Baud | Khi dùng riêng module với divisor nguyên bằng 0, xóa counter/accumulator và ngừng tick. APB vẫn từ chối cấu hình divisor 0. |
| Kết nối | Đặt tên rõ các output và bit RFU không dùng. Chuẩn hóa newline. Lint chạy với `-Wall`, không thêm chỉ thị tắt cảnh báo. |

## Quy ước được giữ theo thiết kế nguồn

- TXDATA với PSTRB=0000 là no-op, không báo E3. Lựa chọn này theo bộ tính tương tác trong HTML. Bảng mô tả E3 chưa ghi ngoại lệ này.
- Ghi BAUD khi idle đổi divisor và giữ pha bộ đếm/accumulator. Bộ so sánh `>=` xử lý trường hợp divisor mới nhỏ hơn counter hiện tại.
- RX lấy mẫu khi bộ đếm 0…15 bằng 8 và các mẫu cách nhau 16 tick. Nếu đánh số sự kiện tick từ 1, đây là sự kiện thứ 9 sau khi khởi tạo đếm. Đây là quy ước hiện thực của nguồn Drive.
- Đọc RXDATA trùng `rx_valid` trả byte cũ qua APB và giữ byte mới cùng cờ lỗi tương ứng. Khi chưa đọc và nhận thêm byte, giữ byte cũ và đặt overrun.
- Cấu hình parity_type=0 là odd, parity_type=1 là even. Reset cấu hình mặc định 5N1, BAUD=0x900D, clock danh định 25 MHz.
- Mỗi chiều có bộ đệm một byte. RTS báo chưa sẵn sàng khi RXDATA chưa được đọc hoặc RX disable/reset. CTS chặn bắt đầu khung mới.

## Kiểm tra và giới hạn

`reports/checks.json` và các log kèm theo ghi kết quả của từng lệnh thực thi.

Kết quả cuối ngày 16/11/2026 đạt 18/18 lệnh kiểm tra. Cả hai top đạt 0 cảnh báo lint và Yosys báo 0 problems, không có latch. Mô phỏng top đạt 36/36 nhóm, kiểm tra baud báo 0 lỗi, RX tolerance đạt 480/480 trường hợp, hồi quy đạt 86/86 phép kiểm tra, SPI đạt 70.822 phép kiểm tra với 101 giao dịch và wrapper đạt 87 phép kiểm tra với 61 khung. Các số lượng có đơn vị khác nhau và được ghi riêng theo từng bộ.

- Biên dịch Verilog-2001 và lint Verilator `-Wall` cho cả hai top.
- Tổng hợp Yosys generic `-noabc` và kiểm tra cấu trúc cho cả hai top.
- Bộ top-level có 36 nhóm, gồm ma trận 24 định dạng, 10 baud, lỗi RX, CTS/RTS, IRQ, reset và lỗi APB.
- Bộ baud kiểm tra tốc độ, chu kỳ phân số và các biên divisor.
- Bộ SPI và wrapper kiểm tra truyền lệnh, dữ liệu và trạng thái.
- Test hồi quy riêng kiểm tra reset APB, đủ 16 giá trị PSTRB khi TX bận, E1…E5, cửa sổ launch pending, đọc RX trùng byte mới, overrun và ưu tiên reset.
- Quét 480 tình huống RX 8E2 với 10 baud, lệch tốc độ −2/0/+2%, 8 pha ban đầu và hai dữ liệu 0x55/0xAA. Độ lệch được tính so với baud thực tế tạo bởi divisor.

Các kết quả mô phỏng áp dụng cho những tình huống đã chạy. Tối ưu diện tích và timing trên Sky130 cần kết quả tổng hợp ánh xạ thư viện, STA và place-and-route. Bản này chưa được xác nhận bằng các bước vật lý đó hoặc phép chứng minh hình thức toàn bộ trạng thái.
