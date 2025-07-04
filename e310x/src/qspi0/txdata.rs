#[doc = "Register `txdata` reader"]
pub type R = crate::R<TxdataSpec>;
#[doc = "Register `txdata` writer"]
pub type W = crate::W<TxdataSpec>;
#[doc = "Field `data` reader - Transmit data"]
pub type DataR = crate::FieldReader;
#[doc = "Field `data` writer - Transmit data"]
pub type DataW<'a, REG> = crate::FieldWriter<'a, REG, 8>;
#[doc = "Field `full` reader - FIFO full flag"]
pub type FullR = crate::BitReader;
#[doc = "Field `full` writer - FIFO full flag"]
pub type FullW<'a, REG> = crate::BitWriter<'a, REG>;
impl R {
    #[doc = "Bits 0:7 - Transmit data"]
    #[inline(always)]
    pub fn data(&self) -> DataR {
        DataR::new((self.bits & 0xff) as u8)
    }
    #[doc = "Bit 31 - FIFO full flag"]
    #[inline(always)]
    pub fn full(&self) -> FullR {
        FullR::new(((self.bits >> 31) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:7 - Transmit data"]
    #[inline(always)]
    pub fn data(&mut self) -> DataW<'_, TxdataSpec> {
        DataW::new(self, 0)
    }
    #[doc = "Bit 31 - FIFO full flag"]
    #[inline(always)]
    pub fn full(&mut self) -> FullW<'_, TxdataSpec> {
        FullW::new(self, 31)
    }
}
#[doc = "Transmit Data Register\n\nYou can [`read`](crate::Reg::read) this register and get [`txdata::R`](R). You can [`reset`](crate::Reg::reset), [`write`](crate::Reg::write), [`write_with_zero`](crate::Reg::write_with_zero) this register using [`txdata::W`](W). You can also [`modify`](crate::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct TxdataSpec;
impl crate::RegisterSpec for TxdataSpec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`txdata::R`](R) reader structure"]
impl crate::Readable for TxdataSpec {}
#[doc = "`write(|w| ..)` method takes [`txdata::W`](W) writer structure"]
impl crate::Writable for TxdataSpec {
    type Safety = crate::Unsafe;
}
#[doc = "`reset()` method sets txdata to value 0"]
impl crate::Resettable for TxdataSpec {}
