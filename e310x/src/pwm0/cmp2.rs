#[doc = "Register `cmp2` reader"]
pub type R = crate::R<Cmp2Spec>;
#[doc = "Register `cmp2` writer"]
pub type W = crate::W<Cmp2Spec>;
#[doc = "Field `value` reader - "]
pub type ValueR = crate::FieldReader<u16>;
#[doc = "Field `value` writer - "]
pub type ValueW<'a, REG> = crate::FieldWriter<'a, REG, 16, u16>;
impl R {
    #[doc = "Bits 0:15"]
    #[inline(always)]
    pub fn value(&self) -> ValueR {
        ValueR::new((self.bits & 0xffff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:15"]
    #[inline(always)]
    pub fn value(&mut self) -> ValueW<'_, Cmp2Spec> {
        ValueW::new(self, 0)
    }
}
#[doc = "Compare Register\n\nYou can [`read`](crate::Reg::read) this register and get [`cmp2::R`](R). You can [`reset`](crate::Reg::reset), [`write`](crate::Reg::write), [`write_with_zero`](crate::Reg::write_with_zero) this register using [`cmp2::W`](W). You can also [`modify`](crate::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct Cmp2Spec;
impl crate::RegisterSpec for Cmp2Spec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`cmp2::R`](R) reader structure"]
impl crate::Readable for Cmp2Spec {}
#[doc = "`write(|w| ..)` method takes [`cmp2::W`](W) writer structure"]
impl crate::Writable for Cmp2Spec {
    type Safety = crate::Unsafe;
}
#[doc = "`reset()` method sets cmp2 to value 0"]
impl crate::Resettable for Cmp2Spec {}
