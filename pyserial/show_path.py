"""
八字形路径生成和可视化验证脚本
用于验证STM32 C代码中的PathTracking_GenTestPath()函数
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math

def generate_figure8_path():
    """
    生成八字形路径 与C代码逻辑完全一致
    
    关键点：
    O1(2000,600) - 圆心1
    O2(1000,600) - 圆心2
    A(1000,1000) - 起点
    B(2000, 200)
    C(2000,1000)
    D(1000,200)
    
    路径顺序：A->B->C->D->A (循环4次)
    """
    
    # 关键点定义
    O1_x, O1_y = 2000, 600  # 圆心1
    O2_x, O2_y = 1000, 600  # 圆心2
    A_x, A_y = 1000, 1000   # 起点A
    B_x, B_y = 2000, 200    # 点B
    C_x, C_y = 2000, 1000   # 点C
    D_x, D_y = 1000, 200    # 点D
    radius = 400             # 圆弧半径
    step = 50                # 路径点间距
    
    path_x = []
    path_y = []
    
    # 循环4次八字形路径
    for loop in range(4):
        print(f"\n=== 循环 {loop + 1} ===")
        
        # 段1：从A直线到B
        dx_AB = B_x - A_x
        dy_AB = B_y - A_y
        dist_AB = math.sqrt(dx_AB**2 + dy_AB**2)
        num_points_AB = int(dist_AB / step)
        
        print(f"段1 (A->B): 距离={dist_AB:.1f}mm, 点数={num_points_AB}")
        
        for i in range(num_points_AB + 1):
            t = i / num_points_AB if num_points_AB > 0 else 0
            x = A_x + int(dx_AB * t)
            y = A_y + int(dy_AB * t)
            path_x.append(x)
            path_y.append(y)
        
        # 段2：以O1为圆心，半径400mm，从B逆时针到C（半圆，180度）
        angle_B = math.atan2(B_y - O1_y, B_x - O1_x)
        angle_C = math.atan2(C_y - O1_y, C_x - O1_x)
        
        arc_length_BC = math.pi * radius  # 半圆弧长
        num_points_BC = int(arc_length_BC / step)
        
        print(f"段2 (B->C): 弧长={arc_length_BC:.1f}mm, 点数={num_points_BC}")
        print(f"  angle_B={math.degrees(angle_B):.1f}°, angle_C={math.degrees(angle_C):.1f}°")
        
        for i in range(1, num_points_BC + 1):
            t = i / num_points_BC
            angle = angle_B + math.pi * t  # 逆时针旋转180度
            x = O1_x + int(radius * math.cos(angle))
            y = O1_y + int(radius * math.sin(angle))
            path_x.append(x)
            path_y.append(y)
        
        # 段3：从C直线到D
        dx_CD = D_x - C_x
        dy_CD = D_y - C_y
        dist_CD = math.sqrt(dx_CD**2 + dy_CD**2)
        num_points_CD = int(dist_CD / step)
        
        print(f"段3 (C->D): 距离={dist_CD:.1f}mm, 点数={num_points_CD}")
        
        for i in range(1, num_points_CD + 1):
            t = i / num_points_CD
            x = C_x + int(dx_CD * t)
            y = C_y + int(dy_CD * t)
            path_x.append(x)
            path_y.append(y)
        
        # 段4：以O2为圆心，半径400mm，从D顺时针到A（半圆，180度）
        angle_D = math.atan2(D_y - O2_y, D_x - O2_x)
        angle_A = math.atan2(A_y - O2_y, A_x - O2_x)
        
        arc_length_DA = math.pi * radius  # 半圆弧长
        num_points_DA = int(arc_length_DA / step)
        
        print(f"段4 (D->A): 弧长={arc_length_DA:.1f}mm, 点数={num_points_DA}")
        print(f"  angle_D={math.degrees(angle_D):.1f}°, angle_A={math.degrees(angle_A):.1f}°")
        
        for i in range(1, num_points_DA + 1):
            t = i / num_points_DA
            angle = angle_D - math.pi * t  # 顺时针旋转180度
            x = O2_x + int(radius * math.cos(angle))
            y = O2_y + int(radius * math.sin(angle))
            path_x.append(x)
            path_y.append(y)
    
    return path_x, path_y, {
        'O1': (O1_x, O1_y),
        'O2': (O2_x, O2_y),
        'A': (A_x, A_y),
        'B': (B_x, B_y),
        'C': (C_x, C_y),
        'D': (D_x, D_y),
        'radius': radius
    }


def visualize_path(path_x, path_y, key_points):
    """可视化路径"""
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # 绘制路径
    ax.plot(path_x, path_y, 'b-', linewidth=1.5, label='路径轨迹', alpha=0.7)
    ax.plot(path_x[0], path_y[0], 'go', markersize=12, label='起点A', zorder=5)
    
    # 绘制关键点
    O1 = key_points['O1']
    O2 = key_points['O2']
    A = key_points['A']
    B = key_points['B']
    C = key_points['C']
    D = key_points['D']
    radius = key_points['radius']
    
    # 绘制圆心和圆弧
    circle1 = Circle(O1, radius, fill=False, edgecolor='red', linestyle='--', linewidth=1, label='圆1 (O1)')
    circle2 = Circle(O2, radius, fill=False, edgecolor='green', linestyle='--', linewidth=1, label='圆2 (O2)')
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    
    # 标注关键点
    ax.plot(*O1, 'r^', markersize=10, label='O1')
    ax.plot(*O2, 'g^', markersize=10, label='O2')
    ax.plot(*B, 'mo', markersize=8, label='B')
    ax.plot(*C, 'co', markersize=8, label='C')
    ax.plot(*D, 'yo', markersize=8, label='D')
    
    # 添加文字标注
    ax.text(O1[0], O1[1]+80, f'O1({O1[0]},{O1[1]})', ha='center', fontsize=10, color='red')
    ax.text(O2[0], O2[1]+80, f'O2({O2[0]},{O2[1]})', ha='center', fontsize=10, color='green')
    ax.text(A[0]-100, A[1], f'A({A[0]},{A[1]})', ha='center', fontsize=10, color='green')
    ax.text(B[0]+100, B[1], f'B({B[0]},{B[1]})', ha='center', fontsize=10, color='magenta')
    ax.text(C[0]+100, C[1], f'C({C[0]},{C[1]})', ha='center', fontsize=10, color='cyan')
    ax.text(D[0]-100, D[1], f'D({D[0]},{D[1]})', ha='center', fontsize=10, color='yellow')
    
    # 设置坐标轴
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title('八字形路径验证 (循环4次)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    return fig


def analyze_path(path_x, path_y):
    """分析路径统计信息"""
    
    print(f"\n{'='*50}")
    print("路径统计信息")
    print(f"{'='*50}")
    print(f"总路径点数: {len(path_x)}")
    
    # 计算总路径长度
    total_length = 0
    for i in range(1, len(path_x)):
        dx = path_x[i] - path_x[i-1]
        dy = path_y[i] - path_y[i-1]
        total_length += math.sqrt(dx**2 + dy**2)
    
    print(f"总路径长度: {total_length:.1f} mm = {total_length/1000:.2f} m")
    print(f"平均每个循环长度: {total_length/4:.1f} mm")
    
    # X,Y范围
    print(f"\nX范围: [{min(path_x)}, {max(path_x)}] mm")
    print(f"Y范围: [{min(path_y)}, {max(path_y)}] mm")
    
    # 检查路径点间距
    distances = []
    for i in range(1, len(path_x)):
        dx = path_x[i] - path_x[i-1]
        dy = path_y[i] - path_y[i-1]
        dist = math.sqrt(dx**2 + dy**2)
        distances.append(dist)
    
    print(f"\n路径点间距统计:")
    print(f"  平均: {np.mean(distances):.2f} mm")
    print(f"  最小: {np.min(distances):.2f} mm")
    print(f"  最大: {np.max(distances):.2f} mm")
    print(f"  标准差: {np.std(distances):.2f} mm")


def main():
    """主函数"""
    
    print("="*60)
    print("八字形路径生成和验证")
    print("="*60)
    
    # 生成路径
    path_x, path_y, key_points = generate_figure8_path()
    
    # 统计分析
    analyze_path(path_x, path_y)
    
    # 可视化
    fig = visualize_path(path_x, path_y, key_points)
    
    # 保存图片
    output_file = 'figure8_path.png'
    fig.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n路径图已保存到: {output_file}")
    
    # 显示图形
    plt.show()
    
    print("\n验证完成！")


if __name__ == "__main__":
    main()
