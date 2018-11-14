/*
 * Copyright 2018 Cog Systems Pty Ltd. All rights reserved.
 *
 * Use of this source code is governed by the 3-Clause BSD license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <libfdt.h>

#define HAS_DEVICE_TREE 1

/* allow space for two clusters but default to uniprocessor */
static int max_clusters = 2;
static zbi_cpu_config_t cpu_config = {
    .cluster_count = 1,
    .clusters = {
        {
            .cpu_count = 1,
        },
        {
            .cpu_count = 0,
        },
    },
};

static int num_mem_configs;
static zbi_mem_range_t mem_config[16];
static dcfg_simple_t uart_driver;
static dcfg_arm_gicv2_driver_t gicv2_driver = {
    .gicd_offset = 0x1000,
    .gicc_offset = 0x2000,
    .ipi_base = 2,
};
static dcfg_arm_psci_driver_t psci_driver = {
    .use_hvc = true
};
static dcfg_arm_generic_timer_driver_t timer_driver;

static const zbi_platform_id_t platform_id = {
    .vid = PDEV_VID_GENERIC,
    .pid = PDEV_PID_GENERIC,
    .board_name = "okl4-generic",
};

static struct {
    const char *cmdline;
    size_t cmdline_length;
    uintptr_t initrd_start;
} chosen;

static void append_board_boot_item(zbi_header_t* bootdata)
{
    // add CPU configuration
    append_boot_item(bootdata, ZBI_TYPE_CPU_CONFIG, 0, &cpu_config,
                     sizeof(zbi_cpu_config_t) +
                     sizeof(zbi_cpu_cluster_t) * cpu_config.cluster_count);

    // add memory configuration
    append_boot_item(bootdata, ZBI_TYPE_MEM_CONFIG, 0, &mem_config,
                     sizeof(zbi_mem_range_t) * num_mem_configs);

    // add kernel drivers
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_GIC_V2,
                     &gicv2_driver, sizeof(gicv2_driver));
    if (uart_driver.irq != 0)
        append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_PL011_UART,
                         &uart_driver, sizeof(uart_driver));
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_PSCI,
                     &psci_driver, sizeof(psci_driver));
    if (timer_driver.irq_virt != 0)
        append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER,
                         KDRV_ARM_GENERIC_TIMER, &timer_driver,
                         sizeof(timer_driver));

    // append kernel command line
    if (chosen.cmdline && chosen.cmdline_length) {
        append_boot_item(bootdata, ZBI_TYPE_CMDLINE, 0,
                         chosen.cmdline, chosen.cmdline_length);
    }

    // add platform ID
    append_boot_item(bootdata, ZBI_TYPE_PLATFORM_ID, 0, &platform_id,
                     sizeof(platform_id));
}

static int count_cpu_nodes(void *device_tree)
{
    int num_cpus = 0;
    int offset = -1;
    while (1) {
        offset = fdt_node_offset_by_prop_value(device_tree, offset,
                                               "device_type", "cpu", 4);
        if (offset < 0)
            break;
        num_cpus++;
    }

    return num_cpus;
}

static int cluster_cpu_count(void *device_tree, int cluster_num)
{
    /*
        cpu-map {
            cluster0 {
                core0 {
                    cpu = <&CPU0>;
                };
                core1 {
                    cpu = <&CPU1>;
                };
                core2 {
                    cpu = <&CPU2>;
                };
                core3 {
                    cpu = <&CPU3>;
                };
            };

            cluster1 {
                core0 {
                    cpu = <&CPU4>;
                };
                core1 {
                    cpu = <&CPU5>;
                };
                core2 {
                    cpu = <&CPU6>;
                };
                core3 {
                    cpu = <&CPU7>;
                };
            };
        };
    */
    char cluster_node[] = "/cpus/cpu-map/clusterXX";
    if (cluster_num > 10) {
        cluster_node[sizeof(cluster_node) - 3] = cluster_num / 10 + '0';
        cluster_node[sizeof(cluster_node) - 2] = cluster_num % 10 + '0';
    } else {
        cluster_node[sizeof(cluster_node) - 3] = cluster_num + '0';
        cluster_node[sizeof(cluster_node) - 2] = '\0';
    }
    int offset = fdt_path_offset(device_tree, cluster_node);
    if (offset < 0) {
        /*
         * If there's no cpu-map cluster node and this is the first cluster,
         * just count the total number of cpu nodes instead and assume it's
         * a single cluster.
         */
        if (cluster_num == 0)
            return count_cpu_nodes(device_tree);
        return offset;
    }

    int cpu_num = 0;
    char cpu_node[] = "coreXX";
    while (1) {
        if (cpu_num > 10) {
            cpu_node[sizeof(cpu_node) - 3] = cpu_num / 10 + '0';
            cpu_node[sizeof(cpu_node) - 2] = cpu_num % 10 + '0';
        } else {
            cpu_node[sizeof(cpu_node) - 3] = cpu_num + '0';
            cpu_node[sizeof(cpu_node) - 2] = '\0';
        }
        if (fdt_subnode_offset(device_tree, offset, cpu_node) < 0)
            break;
        cpu_num++;
    }

    return cpu_num;
}

static void add_memory_range(uint32_t type, uint64_t address,
                             uint64_t size)
{
    if (num_mem_configs == countof(mem_config))
        fail("too many memory regions\n");
    mem_config[num_mem_configs].type = type;
    mem_config[num_mem_configs].paddr = address;
    mem_config[num_mem_configs].length = size;
    num_mem_configs++;
}

// Parse the device tree to find our ZBI, kernel command line, and RAM size.
static void *read_device_tree(void* device_tree)
{
    int parent, offset, length, address_cells, size_cells;
    const void *property;
    const uint32_t *data32;
    const uint64_t *data64;
    uint64_t address, size;

    if (fdt_check_header(device_tree) < 0) {
        fail("fdt_check_header failed\n");
    }

    /* cpu count + clusters */
    int cluster_num = 0;
    while (1) {
        int cpu_count = cluster_cpu_count(device_tree, cluster_num);
        if (cpu_count <= 0)
            break;
        if (cluster_num == max_clusters)
            fail("too many cpu clusters\n");
        cpu_config.clusters[cluster_num].cpu_count = cpu_count;
        cluster_num++;
    }
    cpu_config.cluster_count = cluster_num;

    /* /memreserve/ */
    int count = fdt_num_mem_rsv(device_tree);
    for (int i = 0; i < count; i++) {
        if (fdt_get_mem_rsv(device_tree, i, &address, &size) < 0)
            break;
        add_memory_range(ZBI_MEM_RANGE_RESERVED, address, size);
    }

    /* memory */
    /*
        memory@20200000 {
            device_type = "memory";
            reg = <0x0 0x20200000 0x10000000>;
        };
    */
    offset = -1;
    while (1) {
        offset = fdt_node_offset_by_prop_value(device_tree, offset,
                                               "device_type", "memory",
                                               7);
        if (offset < 0)
            break;
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (!property)
            continue;
        data64 = property;
        data32 = property;
        if (length == sizeof(uint32_t) * 4)
            size = fdt64_to_cpu(data64[1]);
        else if (length == sizeof(uint32_t) * 3)
            size = fdt32_to_cpu(data32[2]);
        else
            fail("invalid memory node\n");
        address = fdt64_to_cpu(data64[0]);
        add_memory_range(ZBI_MEM_RANGE_RAM, address, size);
    }

    /* command line */
    offset = fdt_path_offset(device_tree, "/chosen");
    if (offset >= 0) {
        chosen.cmdline = fdt_getprop(device_tree, offset, "bootargs", &length);
        chosen.cmdline_length = length;
        property = fdt_getprop(device_tree, offset, "linux,initrd-start",
                &length);
        if (property) {
            if (length == sizeof(uint64_t)) {
                data64 = property;
                chosen.initrd_start = fdt64_to_cpu(*data64);
            } else {
                data32 = property;
                chosen.initrd_start = fdt32_to_cpu(*data32);
            }
        }
    }

    /* hardware uart */
    /*
        uart@f7113000 {
            compatible = "arm,pl011";
            reg = <0x0 0xf7113000 0x1000>;
            interrupts = <0x0 0x27 0x4>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,pl011");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (property) {
            parent = fdt_parent_offset(device_tree, offset);
            address_cells = fdt_address_cells(device_tree, parent);
            if (address_cells == 1) {
                data32 = property;
                uart_driver.mmio_phys = fdt32_to_cpu(*data32);
            } else {
                data64 = property;
                uart_driver.mmio_phys = fdt64_to_cpu(*data64);
            }
            add_memory_range(ZBI_MEM_RANGE_PERIPHERAL,
                             uart_driver.mmio_phys, 0x1000);
        }
        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (property && length == sizeof(uint32_t) * 3) {
            data32 = property;
            uart_driver.irq = fdt32_to_cpu(data32[1])
                              + (fdt32_to_cpu(data32[0]) ? 16 : 32);
        }
    }

    /* psci method */
    /*
        psci {
            compatible = "arm,psci-1.1", "arm,psci-1.0", "arm,psci-0.2", "arm,psci-0.1", "arm,psci";
            method = "hvc";
            cpu_suspend = <0xc4000001>;
            cpu_off = <0x84000002>;
            cpu_on = <0xc4000003>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,psci");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "method", &length);
        if (property)
            psci_driver.use_hvc = !strcmp(property, "hvc");
    }

    /* timer */
    /*
        timer {
            compatible = "okl,virtual-arm-timer", "arm,armv8-timer", "arm,armv7-timer";
            always-on;
            interrupts = <0x1 0x0 0x4 0x1 0x0 0x4 0x1 0xb 0x104 0x1 0x0 0x4>;
            interrupt-parent = <&intc>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "okl,virtual-arm-timer");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,armv8-timer");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,armv7-timer");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (property && length == sizeof(uint32_t) * 12) {
            data32 = property;
            timer_driver.irq_virt = fdt32_to_cpu(data32[7])
                                    + (fdt32_to_cpu(data32[6]) ? 16 : 32);
        }
    }

    /* gic */
    /*
        interrupt-controller {
            compatible = "okl,hypervisor-vgic", "arm,cortex-a9-gic", "arm,pl390";
            interrupt-controller;
            #interrupt-cells = <0x3>;
            phandle = <0x3>;
            interrupt-parent = <0x3>;
            reg = <0x0 0xf6801000 0x1000 0x0 0xf6802000 0x1000>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "okl,hypervisor-vgic");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,cortex-a15-gic");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,cortex-a9-gic");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,pl390");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (property) {
            uint64_t gicc_address, gicc_size, gicd_address, gicd_size;
            parent = fdt_parent_offset(device_tree, offset);
            address_cells = fdt_address_cells(device_tree, parent);
            size_cells = fdt_address_cells(device_tree, parent);
            if (address_cells == 1) {
                data32 = property;
                gicd_address = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicd_address = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (size_cells == 1) {
                data32 = property;
                gicd_size = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicd_size = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (address_cells == 1) {
                data32 = property;
                gicc_address = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicc_address = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (size_cells == 1) {
                data32 = property;
                gicc_size = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicc_size = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (gicd_address < gicc_address)
                gicv2_driver.mmio_phys = gicd_address;
            else
                gicv2_driver.mmio_phys = gicc_address;
            /*
             * Zircon's GICv2 crashes during initialisation if we just map
             * 4kB each for the GICD and GICC.  As a workaround, we map a
             * 64kB region and assume that both are in that region.  This is
             * true for a real GICv2 and we can control the addresses of the
             * virtual GICv2 to ensure that it's also true for the virtual
             * GICv2.
             */
            gicv2_driver.mmio_phys &= ~(uint64_t)0xfffUL;
            gicv2_driver.gicd_offset = gicd_address - gicv2_driver.mmio_phys;
            gicv2_driver.gicc_offset = gicc_address - gicv2_driver.mmio_phys;
            add_memory_range(ZBI_MEM_RANGE_PERIPHERAL, gicv2_driver.mmio_phys,
                             0x10000);
        }
    }
    if (offset < 0 || !property)
        fail("gic not found in device tree\n");

    // Use the device tree initrd as the ZBI.
    return (void*)chosen.initrd_start;
}
