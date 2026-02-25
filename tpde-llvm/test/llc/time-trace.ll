; NOTE: Do not autogenerate
; SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
; SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

; RUN: tpde-llc --target=x86_64 -o /dev/null --time-trace=%t.json %s
; RUN: FileCheck --input-file=%t.json %s

; CHECK: "traceEvents"
; CHECK: TPDE_Analysis
; CHECK: TPDE_compute_precise_liveness
; CHECK: TPDE_compute_spills

define i32 @func(i32 %a, i32 %b) {
  %sum = add i32 %a, %b
  ret i32 %sum
}
