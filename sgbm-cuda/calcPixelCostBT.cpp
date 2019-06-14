/*
 For each pixel row1[x], max(maxD, 0) <= minX <= x < maxX <= width - max(0, -minD),
 and for each disparity minD<=d<maxD the function
 computes the cost (cost[(x-minX)*(maxD - minD) + (d - minD)]), depending on the difference between
 row1[x] and row2[x-d]. The subpixel algorithm from
 "Depth Discontinuities by Pixel-to-Pixel Stereo" by Stan Birchfield and C. Tomasi
 is used, hence the suffix BT.

 the temporary buffer should contain width2*2 elements
 */
static void calcPixelCostBT( const Mat& img1, const Mat& img2, int y,
                            int minD, int maxD, CostType* cost,
                            PixType* buffer, const PixType* tab,
                            int tabOfs, int , int xrange_min = 0, int xrange_max = DEFAULT_RIGHT_BORDER )
{
    int x, c, width = img1.cols, cn = img1.channels();
    int minX1 = std::max(maxD, 0), maxX1 = width + std::min(minD, 0); // 视差计算范围
    int D = maxD - minD, width1 = maxX1 - minX1;
    //This minX1 & maxX2 correction is defining which part of calculatable line must be calculated
    //That is needs of parallel algorithm
    xrange_min = (xrange_min < 0) ? 0: xrange_min;
    xrange_max = (xrange_max == DEFAULT_RIGHT_BORDER) || (xrange_max > width1) ? width1 : xrange_max;
    maxX1 = minX1 + xrange_max;
    minX1 += xrange_min;
    width1 = maxX1 - minX1;
    int minX2 = std::max(minX1 - maxD, 0), maxX2 = std::min(maxX1 - minD, width);
    int width2 = maxX2 - minX2;
    const PixType *row1 = img1.ptr<PixType>(y), *row2 = img2.ptr<PixType>(y);
    PixType *prow1 = buffer + width2*2, *prow2 = prow1 + width*cn*2; // buffer后留下两行后面用

    tab += tabOfs;

    for( c = 0; c < cn*2; c++ )
    {
        prow1[width*c] = prow1[width*c + width-1] =
        prow2[width*c] = prow2[width*c + width-1] = tab[0];
    }

    int n1 = y > 0 ? -(int)img1.step : 0, s1 = y < img1.rows-1 ? (int)img1.step : 0; // n1 是左图的上一行，s1 是左图的下一行
    int n2 = y > 0 ? -(int)img2.step : 0, s2 = y < img2.rows-1 ? (int)img2.step : 0;

    int minX_cmn = std::min(minX1,minX2)-1;
    int maxX_cmn = std::max(maxX1,maxX2)+1;
    minX_cmn = std::max(minX_cmn, 1);
    maxX_cmn = std::min(maxX_cmn, width - 1);
    if( cn == 1 )
    {
        for( x = minX_cmn; x < maxX_cmn; x++ )
        {
            prow1[x] = tab[(row1[x+1] - row1[x-1])*2 + row1[x+n1+1] - row1[x+n1-1] + row1[x+s1+1] - row1[x+s1-1]]; // x 方向sobel滤波
            prow2[width-1-x] = tab[(row2[x+1] - row2[x-1])*2 + row2[x+n2+1] - row2[x+n2-1] + row2[x+s2+1] - row2[x+s2-1]]; // 右图逆序存储

            prow1[x+width] = row1[x]; // 第二行对应位置存储灰度值
            prow2[width-1-x+width] = row2[x];
        }
    }
    else
    {
        for( x = minX_cmn; x < maxX_cmn; x++ )
        {
            prow1[x] = tab[(row1[x*3+3] - row1[x*3-3])*2 + row1[x*3+n1+3] - row1[x*3+n1-3] + row1[x*3+s1+3] - row1[x*3+s1-3]];
            prow1[x+width] = tab[(row1[x*3+4] - row1[x*3-2])*2 + row1[x*3+n1+4] - row1[x*3+n1-2] + row1[x*3+s1+4] - row1[x*3+s1-2]];
            prow1[x+width*2] = tab[(row1[x*3+5] - row1[x*3-1])*2 + row1[x*3+n1+5] - row1[x*3+n1-1] + row1[x*3+s1+5] - row1[x*3+s1-1]];

            prow2[width-1-x] = tab[(row2[x*3+3] - row2[x*3-3])*2 + row2[x*3+n2+3] - row2[x*3+n2-3] + row2[x*3+s2+3] - row2[x*3+s2-3]];
            prow2[width-1-x+width] = tab[(row2[x*3+4] - row2[x*3-2])*2 + row2[x*3+n2+4] - row2[x*3+n2-2] + row2[x*3+s2+4] - row2[x*3+s2-2]];
            prow2[width-1-x+width*2] = tab[(row2[x*3+5] - row2[x*3-1])*2 + row2[x*3+n2+5] - row2[x*3+n2-1] + row2[x*3+s2+5] - row2[x*3+s2-1]];

            prow1[x+width*3] = row1[x*3];
            prow1[x+width*4] = row1[x*3+1];
            prow1[x+width*5] = row1[x*3+2];

            prow2[width-1-x+width*3] = row2[x*3];
            prow2[width-1-x+width*4] = row2[x*3+1];
            prow2[width-1-x+width*5] = row2[x*3+2];
        }
    }

    memset( cost + xrange_min*D, 0, width1*D*sizeof(cost[0]) );

    buffer -= width-1-maxX2; // 回退minD - 1步，不过要乘上视差数
    cost -= (minX1-xrange_min)*D + minD; // simplify the cost indices inside the loop

    for( c = 0; c < cn*2; c++, prow1 += width, prow2 += width )
    {
        int diff_scale = c < cn ? 0 : 2;

        // precompute
        //   v0 = min(row2[x-1/2], row2[x], row2[x+1/2]) and
        //   v1 = max(row2[x-1/2], row2[x], row2[x+1/2]) and
        // 其实应该是：v0 = min((row2[x-1] + row2[x])/2, row2[x], (row2[x] + row2[x + 1])/2)
        // 和        v1 = max((row2[x-1] + row2[x])/2, row2[x], (row2[x] + row2[x + 1])/2)

        // 如官方注释，开始计算B.T. metrics，理论可以参考函数前注释给出的论文Depth Discontinuities by Pixel-to-Pixel Stereo
        // 简单来说，就是对于左图像素x和在某个视差值对应的右图像素x-d，不再如SAD一样单纯计算I_l(x) - I_r(x-d),
        // 而是先拟合下右图像素x-d左右 亚像素精度的视差值，即(row2[x-d-1] + row2[x-d])/2和(row2[x-d] + row2[x-d+1])/2，
        // 再计算左图像素与这三者的差并取最小值，即下面的c0；对于左图像素做同样的处理，得到下面的c1
        for( x = width-1-maxX2; x < width-1- minX2; x++ ) // 预先计算出右图的结果并保存
        {
            int v = prow2[x];
            int vl = x > 0 ? (v + prow2[x-1])/2 : v;
            int vr = x < width-1 ? (v + prow2[x+1])/2 : v;
            int v0 = std::min(vl, vr); v0 = std::min(v0, v);
            int v1 = std::max(vl, vr); v1 = std::max(v1, v);
            buffer[x] = (PixType)v0;          // 第一行存最小值
            buffer[x + width2] = (PixType)v1; // 第二行存最大值
        }

        for( x = minX1; x < maxX1; x++ ) // 针对左图像素依次计算cost
        {
            int u = prow1[x];
            int ul = x > 0 ? (u + prow1[x-1])/2 : u;
            int ur = x < width-1 ? (u + prow1[x+1])/2 : u;
            int u0 = std::min(ul, ur); u0 = std::min(u0, u);
            int u1 = std::max(ul, ur); u1 = std::max(u1, u);
            {
                for( int d = minD; d < maxD; d++ )
                {
                    int v = prow2[width-x-1 + d]; // 之前右图是逆序存储，现在需要逆序取出，并加上视差值
                    int v0 = buffer[width-x-1 + d]; // 最小值
                    int v1 = buffer[width-x-1 + d + width2]; // 最大值
                    int c0 = std::max(0, u - v1); c0 = std::max(c0, v0 - u);
                    int c1 = std::max(0, v - u1); c1 = std::max(c1, u0 - v);
                    // cost计算。与原论文不同的是，opencv的B.T. metrics包含了两个部分，
                    // 一部分为prow1和prow2第一行所存储的左右图的sobel滤波结果的B.T. metrics，
                    // 一部分为prow1和prow2第二行所存储的左右图的灰度值的B.T. metrics
                    cost[x*D + d] = (CostType)(cost[x*D+d] + (std::min(c0, c1) >> diff_scale));
                }
            }
        }
    }
}