# Mở file gốc để đọc và file mới để ghi
input_file = "UTE_MAP\MAP_NGHICH\HD_MAPsai.txt"  # Đổi tên file nếu cần
output_file = "UTE_MAP\MAP_NGHICH\HD_MAP1.txt"

with open(input_file, "r") as infile, open(output_file, "w") as outfile:
    # Đọc toàn bộ nội dung file
    data = infile.read()
    
    # Thay thế "/n" bằng "\n" để xuống dòng
    fixed_data = data.replace("/n", "\n")
    
    # Ghi nội dung đã sửa vào file mới
    outfile.write(fixed_data)

print(f"Định dạng đã được sửa và lưu vào file {output_file}.")
