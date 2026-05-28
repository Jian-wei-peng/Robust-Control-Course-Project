# 演示 GIF 动图

3 个动图,共 ~1.1 MB,用于 PowerPoint / Keynote 现场演示。

| 文件 | 大小 | 时长 | 内容 |
|------|------|------|------|
| `piecewise_side_by_side.gif` | 391 KB | ~17 s | 左 LQR / 右 LQR+DOB 双面板,机器人逐帧前进,顶部显示当前 $\tau_d$,误差实时刷新 |
| `step_traj_and_error.gif` | 382 KB | ~17 s | 左侧轨迹动 + 右侧 \|err\| vs t 曲线同步生长,标出 t=8s 阶跃和 LQR 0.11m 稳态线 |
| `dob_disturbance_estimate.gif` | 335 KB | ~17 s | 上下两通道,DOB 估计追踪真值 $d$,显示每次切换后的瞬态 |

## 在 PowerPoint 里使用

1. **插入 → 图片 → 此设备**,选 `.gif` 文件
2. PPT **播放幻灯片时会自动循环动画**(不放映模式只显示首帧)
3. 推荐放在以下幻灯片:
   - `piecewise_side_by_side.gif` → 替换 **Slide 14: Trajectory tracking under piecewise**
   - `step_traj_and_error.gif` → 替换 **Slide 15: Step disturbance position error**
   - `dob_disturbance_estimate.gif` → 替换 **Slide 16: Disturbance estimate vs truth**

## 在 Keynote 里使用

直接拖入幻灯片即可,默认会自动播放。在右侧"格式 → 动画"里可以调"循环"或"反向播放"。

## LaTeX/Beamer 不能直接嵌入

Beamer 输出 PDF,PDF 标准不支持 GIF 动画(只会显示首帧)。两种解决方案:

- **方案 A(推荐)**:就保留 Beamer 当备用静态版本,正式答辩用 PowerPoint 嵌 GIF
- **方案 B**:用 `\usepackage{animate}` 把 GIF 拆成 PNG 序列再嵌入,但只在 Adobe Acrobat 里能播,Preview/SumatraPDF 看不到。复杂且不通用,不推荐

## 重新生成

如果改了仿真参数想重新出 GIF:

```bash
# scripts are at /tmp/make_gif{1,2,3}_*.py (preserved with the project)
python3 scripts/make_gif1_piecewise.py
python3 scripts/make_gif2_step.py
python3 scripts/make_gif3_dhat.py
# then optimize:
for f in *.gif; do
  ffmpeg -y -i $f -vf "fps=15,scale=900:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=5" tmp.gif
  mv tmp.gif $f
done
```
