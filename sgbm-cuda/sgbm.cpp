/*
 computes disparity for "roi" in img1 w.r.t. img2 and write it to disp1buf.
 that is, disp1buf(x, y)=d means that img1(x+roi.x, y+roi.y) ~ img2(x+roi.x-d, y+roi.y).
 minD <= d < maxD.
 disp2full is the reverse disparity map, that is:
 disp2full(x+roi.x,y+roi.y)=d means that img2(x+roi.x, y+roi.y) ~ img1(x+roi.x+d, y+roi.y)

 note that disp1buf will have the same size as the roi and
 disp2full will have the same size as img1 (or img2).
 On exit disp2buf is not the final disparity, it is an intermediate result that becomes
 final after all the tiles are processed.

 the disparity in disp1buf is written with sub-pixel accuracy
 (4 fractional bits, see StereoSGBM::DISP_SCALE),
 using quadratic interpolation, while the disparity in disp2buf
 is written as is, without interpolation.

 disp2cost also has the same size as img1 (or img2).
 It contains the minimum current cost, used to find the best disparity, corresponding to the minimal cost.
 */
static void computeDisparitySGBM( const Mat& img1, const Mat& img2,
                                 Mat& disp1, const StereoSGBMParams& params,
                                 Mat& buffer )
{
    const int ALIGN = 16;
    const int DISP_SHIFT = StereoMatcher::DISP_SHIFT;
    const int DISP_SCALE = (1 << DISP_SHIFT);
    const CostType MAX_COST = SHRT_MAX;

    int minD = params.minDisparity, maxD = minD + params.numDisparities;
    Size SADWindowSize;
    SADWindowSize.width = SADWindowSize.height = params.SADWindowSize > 0 ? params.SADWindowSize : 5;
    int ftzero = std::max(params.preFilterCap, 15) | 1;
    int uniquenessRatio = params.uniquenessRatio >= 0 ? params.uniquenessRatio : 10;
    int disp12MaxDiff = params.disp12MaxDiff > 0 ? params.disp12MaxDiff : 1; // consisit check 所允许的最大差异
    int P1 = params.P1 > 0 ? params.P1 : 2, P2 = std::max(params.P2 > 0 ? params.P2 : 5, P1+1);
    int k, width = disp1.cols, height = disp1.rows;
    int minX1 = std::max(maxD, 0), maxX1 = width + std::min(minD, 0);
    int D = maxD - minD, width1 = maxX1 - minX1; // 视差值的有效搜索范围
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP*DISP_SCALE;
    int SW2 = SADWindowSize.width/2, SH2 = SADWindowSize.height/2;
    bool fullDP = params.mode == StereoSGBM::MODE_HH;
    int npasses = fullDP ? 2 : 1;
    const int TAB_OFS = 256*4, TAB_SIZE = 256 + TAB_OFS*2;
    PixType clipTab[TAB_SIZE];

    // 同stereoBM, xsobel索引表
    for( k = 0; k < TAB_SIZE; k++ )
        clipTab[k] = (PixType)(std::min(std::max(k - TAB_OFS, -ftzero), ftzero) + ftzero);

    if( minX1 >= maxX1 )
    {
        disp1 = Scalar::all(INVALID_DISP_SCALED);
        return;
    }

    CV_Assert( D % 16 == 0 );

    // NR - the number of directions. the loop on x below that computes Lr assumes that NR == 8.
    // if you change NR, please, modify the loop as well.
    int D2 = D+16, NRD2 = NR2*D2;

    // the number of L_r(.,.) and min_k L_r(.,.) lines in the buffer:
    // for 8-way dynamic programming we need the current row and
    // the previous row, i.e. 2 rows in total
    const int NLR = 2;
    const int LrBorder = NLR - 1;

    // for each possible stereo match (img1(x,y) <=> img2(x-d,y))
    // we keep pixel difference cost (C) and the summary cost over NR directions (S).
    // we also keep all the partial costs for the previous line L_r(x,d) and also min_k L_r(x, k)
    size_t costBufSize = width1*D;
    size_t CSBufSize = costBufSize*(fullDP ? height : 1); // cost aggregation 采用8个方向的话则需要WHD大小的内存
    size_t minLrSize = (width1 + LrBorder*2)*NR2, LrSize = minLrSize*D2; // minLr 存储最优视差对应的最小cost，大小为有效宽度（再加2）乘以8个方向
                                     // Lr 在此基础上需要存储160个视差值所对应的全部cost。
                                     // DP 算法每次只用到上一行或下一行的信息，因此只保留两行的数据，最后乘以NR2
    int hsumBufNRows = SH2*2 + 2; // 一个匹配窗口所包含的行数再加1，方便滑动窗口法每次多算一行
    size_t totalBufSize = (LrSize + minLrSize)*NLR*sizeof(CostType) + // minLr[] and Lr[]
    costBufSize*(hsumBufNRows + 1)*sizeof(CostType) + // hsumBuf, pixdiff // costBufSize * 1 是pixeldiff的内存大小，一行各个像素的所有可能视差值对应的cost
                              // costBufSize * hsumBufNRows 是hsumBuf的内存大小，滑动窗口内各行的像素对应的所有视差值的cost
    CSBufSize*2*sizeof(CostType) + // C, S  // C 和S 都需要保存WHD个计算结果
    width*16*img1.channels()*sizeof(PixType) + // temp buffer for computing per-pixel cost
    width*(sizeof(CostType) + sizeof(DispType)) + 1024; // disp2cost + disp2

    if( buffer.empty() || !buffer.isContinuous() ||
        buffer.cols*buffer.rows*buffer.elemSize() < totalBufSize )
        buffer.reserveBuffer(totalBufSize);

    // summary cost over different (nDirs) directions
    CostType* Cbuf = (CostType*)alignPtr(buffer.ptr(), ALIGN); // C
    CostType* Sbuf = Cbuf + CSBufSize; // S 
    CostType* hsumBuf = Sbuf + CSBufSize;
    CostType* pixDiff = hsumBuf + costBufSize*hsumBufNRows;

    CostType* disp2cost = pixDiff + costBufSize + (LrSize + minLrSize)*NLR; // 预留给hsumBuf, Lr[]和minLr[]
    DispType* disp2ptr = (DispType*)(disp2cost + width);
    PixType* tempBuf = (PixType*)(disp2ptr + width);

    // add P2 to every C(x,y). it saves a few operations in the inner loops
    for(k = 0; k < (int)CSBufSize; k++ )
        Cbuf[k] = (CostType)P2;

    for( int pass = 1; pass <= npasses; pass++ )
    {
        int x1, y1, x2, y2, dx, dy;

        if( pass == 1 ) // 正向遍历，先计算正向遍历可以计算的4个方向的Lr和minLr，并保存结果在C和S中
        {
            y1 = 0; y2 = height; dy = 1;
            x1 = 0; x2 = width1; dx = 1;
        }
        else // 逆序遍历
        {
            y1 = height-1; y2 = -1; dy = -1;
            x1 = width1-1; x2 = -1; dx = -1;
        }

        // 处理方向变化时重新分配指针
        CostType *Lr[NLR]={0}, *minLr[NLR]={0};

        for( k = 0; k < NLR; k++ )
        {
            // shift Lr[k] and minLr[k] pointers, because we allocated them with the borders,
            // and will occasionally use negative indices with the arrays
            // we need to shift Lr[k] pointers by 1, to give the space for d=-1.
            // however, then the alignment will be imperfect, i.e. bad for SSE,
            // thus we shift the pointers by 8 (8*sizeof(short) == 16 - ideal alignment)
            Lr[k] = pixDiff + costBufSize + LrSize*k + NRD2*LrBorder + 8;
            // 8是为了sse优化，NRD2 * LrBorder 是边界一个像素8个方向的所有视差值对应的大小
            memset( Lr[k] - LrBorder*NRD2 - 8, 0, LrSize*sizeof(CostType) ); 
            minLr[k] = pixDiff + costBufSize + LrSize*NLR + minLrSize*k + NR2*LrBorder;
            memset( minLr[k] - LrBorder*NR2, 0, minLrSize*sizeof(CostType) );
        }

        for( int y = y1; y != y2; y += dy )
        {
            int x, d;
            DispType* disp1ptr = disp1.ptr<DispType>(y); // 视差图第y行
            CostType* C = Cbuf + (!fullDP ? 0 : y*costBufSize); // 跳过前y行
            CostType* S = Sbuf + (!fullDP ? 0 : y*costBufSize);

            if( pass == 1 ) // compute C on the first pass, and reuse it on the second pass, if any.
            {
                int dy1 = y == 0 ? 0 : y + SH2, dy2 = y == 0 ? SH2 : dy1;

                for( k = dy1; k <= dy2; k++ ) // y = 0，先把滑动窗口第上半部分计算好存储在hsumBuf中
                                              // y != 0，计算y + SH2行
                {
                    // 类似于stereoBM，保存并复用计算结果
                    CostType* hsumAdd = hsumBuf + (std::min(k, height-1) % hsumBufNRows)*costBufSize;

                    if( k < height )
                    {
                        // 计算第k行，每个像素的所有视差值的cost
                        calcPixelCostBT( img1, img2, k, minD, maxD, pixDiff, tempBuf, clipTab, TAB_OFS, ftzero );

                        memset(hsumAdd, 0, D*sizeof(CostType));
                        for( x = 0; x <= SW2*D; x += D ) // 累加左半边窗口的cost
                        {
                            int scale = x == 0 ? SW2 + 1 : 1; // 第一行要加SW2 + 1次，因为第一个窗口从第一列开始，取不到前面的列就需要累加第一列
                                                              // 后面相减 (pixSub) 也是一样的
                            for( d = 0; d < D; d++ )
                                hsumAdd[d] = (CostType)(hsumAdd[d] + pixDiff[x + d]*scale); // 累加当前行前SW2 + 1个像素每个视差值的cost
                        }

                        if( y > 0 )
                        {
                            // 滑动窗口法
                            const CostType* hsumSub = hsumBuf + (std::max(y - SH2 - 1, 0) % hsumBufNRows)*costBufSize; // 所要减去的对应行
                            const CostType* Cprev = !fullDP || y == 0 ? C : C - costBufSize; // 上一行的C

                            for( x = D; x < width1*D; x += D )
                            {
                                const CostType* pixAdd = pixDiff + std::min(x + SW2*D, (width1-1)*D); // 所要加上和减去的列
                                const CostType* pixSub = pixDiff + std::max(x - (SW2+1)*D, 0);
                                {
                                    for( d = 0; d < D; d++ )
                                    {
                                        // 滑动窗口法
                                        int hv = hsumAdd[x + d] = (CostType)(hsumAdd[x - D + d] + pixAdd[d] - pixSub[d]);
                                        // 当前行某个像素视差为d的cost为上一行对应的cost加上该像素匹配的cost
                                        C[x + d] = (CostType)(Cprev[x + d] + hv - hsumSub[x + d]);
                                    }
                                }
                            }
                        }
                        else
                        {
                            for( x = D; x < width1*D; x += D ) // 开始对第0行进行计算，直到最后一个像素
                            {
                                const CostType* pixAdd = pixDiff + std::min(x + SW2*D, (width1-1)*D); // 前面累加了当前行前SW2个像素，现在从第SW2 + 1个像素开始累加
                                const CostType* pixSub = pixDiff + std::max(x - (SW2+1)*D, 0); // 从-SW2列开始减去

                                for( d = 0; d < D; d++ ) // 当前像素y = 0, x = x / D 的所有视差值
                                    hsumAdd[x + d] = (CostType)(hsumAdd[x - D + d] + pixAdd[d] - pixSub[d]); // 滑动窗口，列方向上减去上一列，加上下一列
                            }
                        }
                    }

                    if( y == 0 ) // 针对第一行，初始化C
                                 // 后面行的C 都需要前一行的信息
                    {
                        int scale = k == 0 ? SH2 + 1 : 1; // 第一行需要多加8次，因为后面的窗口从第一行开始，取不到前面的行就需要累加第一行的值
                                                          // 后面相减 (pixSub) 也是一样的
                        for( x = 0; x < width1*D; x++ )
                            C[x] = (CostType)(C[x] + hsumAdd[x]*scale);
                    }
                }

                // also, clear the S buffer
                for( k = 0; k < width1*D; k++ )
                    S[k] = 0;
            }

            // clear the left and the right borders
            // 置0处理，一方面防止后面的数组越界，一方面方便后续累加
            memset( Lr[0] - NRD2*LrBorder - 8, 0, NRD2*LrBorder*sizeof(CostType) );
            memset( Lr[0] + width1*NRD2 - 8, 0, NRD2*LrBorder*sizeof(CostType) );
            memset( minLr[0] - NR2*LrBorder, 0, NR2*LrBorder*sizeof(CostType) );
            memset( minLr[0] + width1*NR2, 0, NR2*LrBorder*sizeof(CostType) );

            /*
             [formula 13 in the paper]
             compute L_r(p, d) = C(p, d) +
             min(L_r(p-r, d),
             L_r(p-r, d-1) + P1,
             L_r(p-r, d+1) + P1,
             min_k L_r(p-r, k) + P2) - min_k L_r(p-r, k)
             where p = (x,y), r is one of the directions.
             we process all the directions at once:
             0: r=(-dx, 0)
             1: r=(-1, -dy)
             2: r=(0, -dy)
             3: r=(1, -dy)
             4: r=(-2, -dy)
             5: r=(-1, -dy*2)
             6: r=(1, -dy*2)
             7: r=(2, -dy)
             */
            
            // y 处理完，开始处理x
            for( x = x1; x != x2; x += dx ) // 注意pass = npass (2) 时为逆序
            {
                int xm = x*NR2, xd = xm*D2;

                // 待处理的4个方向的前一个像素与当前像素视差值相差大于2时的cost
                // 待处理的4个方向对英语官方注释中的0～3。注意dx, dy的符号
                int delta0 = minLr[0][xm - dx*NR2] + P2, delta1 = minLr[1][xm - NR2 + 1] + P2;
                int delta2 = minLr[1][xm + 2] + P2, delta3 = minLr[1][xm + NR2 + 3] + P2;

                // 待处理的4个方向的Lr的指针
                // NRD2为视差范围*方向数，减去后对应前一个像素所有8个方向的所有视差值所对应的Lr值。Lr从第0个方向开始存储
                // D2的值为视差范围，每加上一个D2就意味着跳过一个方向
                // 与前面的delta对应，但数组下标差D2倍，因为minLr存储的是8个方向的Lr的8个最小值，而Lr存储 8*视差范围 的所有值
                /* 1 2 3
                   0 x 0
                   1 2 3 */
                CostType* Lr_p0 = Lr[0] + xd - dx*NRD2; // pass = 1, dx = 1，为当前行上一列像素Lr的第0个方向
                                                        // pass = 2, dx = -1，为当前行下一列像素Lr的第0个方向
                CostType* Lr_p1 = Lr[1] + xd - NRD2 + D2; // pass = 1，上一行上一列像素的Lr的第1个方向
                                                          // pass = 2，下一行上一列像素的Lr的第1个方向
                CostType* Lr_p2 = Lr[1] + xd + D2*2; // pass = 1，上一行当前列像素Lr的第2个方向
                                                     // pass = 2，下一行当前列像素Lr的地2个方向
                CostType* Lr_p3 = Lr[1] + xd + NRD2 + D2*3; // pass = 1，上一行下一列像素Lr的第3个方向
                                                            // pass = 2，下一行下一列像素Lr的第3个方向

                // 解释：pass = 1时好理解。当pass = 2时，从最后一行的最后一列开始逆序处理。由于最后一行只能在正向遍历时计算出前4个方向的Lr值，
                // 因此再处理倒数第二行时，只能访问下一行当前4个方向的Lr。
                // 当pass = 2时，第一个方向Lr_p0，正序处理时它应该是上一列的像素，而逆序处理时应该是下一列的像素，因此由dx控制方向。
                // 而行方向上上下行的差异在处理完分别交换Lr, minLr内的指针时就处理好了。
                // 之后，倒数第二行开始逆序处理时，其计算结果也更新在了Lr的前4个方向中，相当于把正序处理时的结果覆盖了。
                // 但因为用不到了，只要S能够正确累加被覆盖也没有关系
                Lr_p0[-1] = Lr_p0[D] = Lr_p1[-1] = Lr_p1[D] =
                Lr_p2[-1] = Lr_p2[D] = Lr_p3[-1] = Lr_p3[D] = MAX_COST; // 将多申请的前后两个视差值置为最大cost，为后面循环中的数组越界做准备

                CostType* Lr_p = Lr[0] + xd; // 将指针调节至当前像素
                const CostType* Cp = C + x*D;
                CostType* Sp = S + x*D;
                {
                    int minL0 = MAX_COST, minL1 = MAX_COST, minL2 = MAX_COST, minL3 = MAX_COST;

                    for( d = 0; d < D; d++ )
                    {
                        // 4个方向的 相同disp的cost 和 不同disp加上惩罚值后的cost 的计算
                        int Cpd = Cp[d], L0, L1, L2, L3;

                        L0 = Cpd + std::min((int)Lr_p0[d], std::min(Lr_p0[d-1] + P1, std::min(Lr_p0[d+1] + P1, delta0))) - delta0;
                        L1 = Cpd + std::min((int)Lr_p1[d], std::min(Lr_p1[d-1] + P1, std::min(Lr_p1[d+1] + P1, delta1))) - delta1;
                        L2 = Cpd + std::min((int)Lr_p2[d], std::min(Lr_p2[d-1] + P1, std::min(Lr_p2[d+1] + P1, delta2))) - delta2;
                        L3 = Cpd + std::min((int)Lr_p3[d], std::min(Lr_p3[d-1] + P1, std::min(Lr_p3[d+1] + P1, delta3))) - delta3;

                        // 存储对应的Lr值并获取各个方向的minLr
                        Lr_p[d] = (CostType)L0;
                        minL0 = std::min(minL0, L0);

                        Lr_p[d + D2] = (CostType)L1;
                        minL1 = std::min(minL1, L1);

                        Lr_p[d + D2*2] = (CostType)L2;
                        minL2 = std::min(minL2, L2);

                        Lr_p[d + D2*3] = (CostType)L3;
                        minL3 = std::min(minL3, L3);

                        // 累加到S
                        Sp[d] = saturate_cast<CostType>(Sp[d] + L0 + L1 + L2 + L3);
                    }

                    // 存储minLr
                    minLr[0][xm] = (CostType)minL0;
                    minLr[0][xm+1] = (CostType)minL1;
                    minLr[0][xm+2] = (CostType)minL2;
                    minLr[0][xm+3] = (CostType)minL3;
                }
            }

            if( pass == npasses )
            {
                for( x = 0; x < width; x++ )
                {
                    disp1ptr[x] = disp2ptr[x] = (DispType)INVALID_DISP_SCALED;
                    disp2cost[x] = MAX_COST;
                }

                for( x = width1 - 1; x >= 0; x-- ) // 逆序处理
                {
                    CostType* Sp = S + x*D;
                    int minS = MAX_COST, bestDisp = -1;

                    if( npasses == 1 )
                    {
                        int xm = x*NR2, xd = xm*D2;

                        int minL0 = MAX_COST;
                        int delta0 = minLr[0][xm + NR2] + P2;
                        CostType* Lr_p0 = Lr[0] + xd + NRD2;
                        Lr_p0[-1] = Lr_p0[D] = MAX_COST;
                        CostType* Lr_p = Lr[0] + xd;

                        const CostType* Cp = C + x*D;
                        {
                            for( d = 0; d < D; d++ )
                            {
                                int L0 = Cp[d] + std::min((int)Lr_p0[d], std::min(Lr_p0[d-1] + P1, std::min(Lr_p0[d+1] + P1, delta0))) - delta0;

                                Lr_p[d] = (CostType)L0;
                                minL0 = std::min(minL0, L0);

                                int Sval = Sp[d] = saturate_cast<CostType>(Sp[d] + L0);
                                if( Sval < minS )
                                {
                                    minS = Sval;
                                    bestDisp = d;
                                }
                            }
                            minLr[0][xm] = (CostType)minL0;
                        }
                    }
                    else // 当pass = 2时，从最后一行开始，最后一行上各个像素的最优视差的确定只能参考之前pass = 1时计算的前4个方向
                    {    // 后4个方向的结果需要最后一行计算完后才有
                        {
                            // 遍历寻找最优视差
                            for( d = 0; d < D; d++ )
                            {
                                int Sval = Sp[d];
                                if( Sval < minS )
                                {
                                    minS = Sval;
                                    bestDisp = d;
                                }
                            }
                        }
                    }

                    // 唯一匹配检测。要求除了bestDisp前后各一个视差之外，其余视差值对应的S必须大于minS * 1.x
                    for( d = 0; d < D; d++ )
                    {
                        if( Sp[d]*(100 - uniquenessRatio) < minS*100 && std::abs(bestDisp - d) > 1 )
                            break;
                    }
                    if( d < D ) // 不满足唯一匹配的要求的像素直接跳过
                        continue;
                    d = bestDisp;
                    int _x2 = x + minX1 - d - minD; // _x2为当前像素x在右图上所对应的匹配像素
                    if( disp2cost[_x2] > minS )
                    {
                        // 存储对应的右图像素的视差值和S
                        disp2cost[_x2] = (CostType)minS; 
                        disp2ptr[_x2] = (DispType)(d + minD);
                    }

                    // 当最优视差不为视差搜索范围的首尾时则进行亚像素级别的视差值内查
                    if( 0 < d && d < D-1 )
                    {
                        // do subpixel quadratic interpolation:
                        //   fit parabola into (x1=d-1, y1=Sp[d-1]), (x2=d, y2=Sp[d]), (x3=d+1, y3=Sp[d+1])
                        //   then find minimum of the parabola.
                        int denom2 = std::max(Sp[d-1] + Sp[d+1] - 2*Sp[d], 1);
                        d = d*DISP_SCALE + ((Sp[d-1] - Sp[d+1])*DISP_SCALE + denom2)/(denom2*2);
                    }
                    else
                        d *= DISP_SCALE;
                    disp1ptr[x + minX1] = (DispType)(d + minD*DISP_SCALE); // 存储结果
                }

                for( x = minX1; x < maxX1; x++ )
                {
                    // we round the computed disparity both towards -inf and +inf and check
                    // if either of the corresponding disparities in disp2 is consistent.
                    // This is to give the computed disparity a chance to look valid if it is.
                    // 如官方注释，左右图视差校验。
                    // 经过前面的处理后，对于左图像素x，考虑之前得到的亚像素精度级别的视差d1
                    // 可以获得两个可能的、位于d1前后的最优视差_d = d1 / scale 和 d_=_d+(scale - 1)/scale
                    // 依此获取之前存储的对应的右图像素x - _d和x - d_
                    // 查看二者存储的视差disp2ptr[_x] 和disp2tpr[x_]与当前视差_d和d_的差异
                    // 若大于阈值，则说明consist check失败，当前视差为无效视差
                    int d1 = disp1ptr[x];
                    if( d1 == INVALID_DISP_SCALED )
                        continue;
                    int _d = d1 >> DISP_SHIFT;
                    int d_ = (d1 + DISP_SCALE-1) >> DISP_SHIFT;
                    int _x = x - _d, x_ = x - d_;
                    if( 0 <= _x && _x < width && disp2ptr[_x] >= minD && std::abs(disp2ptr[_x] - _d) > disp12MaxDiff &&
                       0 <= x_ && x_ < width && disp2ptr[x_] >= minD && std::abs(disp2ptr[x_] - d_) > disp12MaxDiff )
                        disp1ptr[x] = (DispType)INVALID_DISP_SCALED;
                }
            }

            // now shift the cyclic buffers
            std::swap( Lr[0], Lr[1] ); // 如此，当pass = 1时，index = 1为上一行，index = 0为当前行
            std::swap( minLr[0], minLr[1] ); // 当pass = 2时，index = 1为下一行，index = 0为当前行
        }
    }
}